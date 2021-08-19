#include "utilities/IOManager.h"

#include "utilities/GameStateManager.hpp"

#include "roboteam_utils/normalize.h"
#include "world/World.hpp"

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

std::optional<proto::UiValues> IOManager::centralServerReceiveLastMessage(){
  bool received = true;
  stx::Result<proto::UiValues, std::string> last_message = stx::Err(std::string(""));

  while(received){
    auto tmp_last_message = central_server_connection->read_next<proto::UiValues>();

    if (tmp_last_message.is_ok()) {
        last_message = std::move(tmp_last_message);
    } else {
      received = false;
      //we don't print the errors as they mark there are no more messages
    }
  }

  if(last_message.is_ok()) {
    return last_message.value();
  }

  return std::nullopt;
}

std::vector<proto::UiValues> IOManager::centralServerReceiveDeltas() {
    bool received = true;
    std::vector<proto::UiValues> messages = {};

    while(received){
        auto tmp_message = central_server_connection->read_next<proto::UiValues>();

        if (tmp_message.is_ok()) {
            messages.emplace_back(std::move(tmp_message.value()));
        } else {
            received = false;
            //we don't print the errors as they mark there are no more messages
        }
    }

    return messages;
}


proto::State IOManager::getState(){
  std::lock_guard<std::mutex> lock(stateMutex);//read lock
  proto::State copy = state;
  return copy;
}
void IOManager::publishAICommand(const proto::AICommand& command) {
  proto::AICommand ai_command;
  ai_command.CopyFrom(command);
  ai_command.mutable_extrapolatedworld()->CopyFrom(getState().command_extrapolated_world()); //TODO: move this responsibility to the AI
  robotCommandPublisher->send(command);
}

void IOManager::centralServerSend(std::vector<proto::Handshake> handshakes) {
  //then, send the current state once
  proto::ModuleState module_state;
  {
    std::lock_guard<std::mutex> lock(stateMutex); //read lock
    module_state.mutable_system_state()->CopyFrom(state);
  }
  for(const auto& handshake : handshakes){
    module_state.mutable_handshakes()->Add()->CopyFrom(handshake);
  }
  central_server_connection->write(module_state,true);
}
}  // namespace rtt::ai::io
