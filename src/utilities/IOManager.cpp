#include <utilities/IOManager.h>

#include <include/roboteam_ai/utilities/GameStateManager.hpp>

#include "roboteam_utils/normalize.h"
#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::io {


IOManager::~IOManager() {
  delete central_server_connection;
  delete settingsPublisher;
  delete robotCommandPublisher;
  delete worldSubscriber;
}

void IOManager::init(int teamId) {
    RTT_INFO("Setting up IO publishers/subscribers")
    worldSubscriber = new proto::Subscriber<proto::State>(proto::WORLD_CHANNEL, &IOManager::handleState, this);

    // set up advertisement to publish robotcommands and settings
    if (teamId == 1) {
        robotCommandPublisher = new proto::Publisher<proto::AICommand>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        robotCommandPublisher = new proto::Publisher<proto::AICommand>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        settingsPublisher = new proto::Publisher<proto::Setting>(proto::SETTINGS_PRIMARY_CHANNEL);
    }
    central_server_connection = new networking::PairReceiver<16970>();
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(proto::State &stateMsg) {
    std::unique_lock<std::mutex> lock(stateMutex); //write lock
    this->state.CopyFrom(stateMsg);
    //TODO: move this to the ai
}

void IOManager::publishSettings(proto::Setting setting) { settingsPublisher->send(setting); }

void IOManager::publishAllRobotCommands(const std::vector<proto::RobotCommand>& robotCommands) {
        proto::AICommand command;
        for(const auto& robotCommand : robotCommands){
          proto::RobotCommand * protoCommand = command.mutable_commands()->Add();
          protoCommand->CopyFrom(robotCommand);
        }
        command.mutable_extrapolatedworld()->CopyFrom(getState().command_extrapolated_world());
        robotCommandPublisher->send(command);
}
void IOManager::handleCentralServerConnection(){
  //first receive any setting changes
  bool received = true;
  int numReceivedMessages = 0;
  while(received){
    auto receivedUIOptions = central_server_connection->read_next<proto::UiSettings>();
    if (receivedUIOptions.is_ok()){
      //TODO: process value
      receivedUIOptions.value().PrintDebugString();
      numReceivedMessages ++;
    }else{
      received = false;
      //we don't print the errors as they mark there are no more messages
    }
  }
  if(numReceivedMessages>0){
    std::cout<<"received " << numReceivedMessages <<" packets from central server"<<std::endl;
  }
  //TODO: actually change settings at the relevant places within our AI
  //TODO: make sure to write/add relevant debug information/visualizations (strategy debug, etc.)
  //then, send the current state once
  proto::ModuleState module_state;
  {
    std::lock_guard<std::mutex> lock(stateMutex); //read lock
    module_state.mutable_system_state()->mutable_state()->CopyFrom(state);
  }
    central_server_connection->write(module_state,true);
}
proto::State IOManager::getState(){
  std::lock_guard<std::mutex> lock(stateMutex);//read lock
  proto::State copy = state;
  return copy;
}
}  // namespace rtt::ai::io
