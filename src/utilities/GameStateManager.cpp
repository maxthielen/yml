#include "utilities/GameStateManager.hpp"

#include <roboteam_utils/Print.h>
#include <include/roboteam_ai/world/World.hpp>

namespace rtt::ai {

proto::SSL_Referee GameStateManager::refMsg;
StrategyManager GameStateManager::strategymanager;
std::mutex GameStateManager::refMsgLock;
int GameStateManager::keeperID;
GameState GameStateManager::interface_gamestate("halt_strategy", "default");
proto::SSL_Referee GameStateManager::getRefereeData() {
    std::lock_guard<std::mutex> lock(refMsgLock);
    return GameStateManager::refMsg;
}

void GameStateManager::setRefereeData(proto::SSL_Referee refMsg, const rtt_world::World* data,
                                      const AISettings& settings) {
    std::lock_guard<std::mutex> lock(refMsgLock);
    GameStateManager::refMsg = refMsg;
    RefCommand cmd;
    if (settings.isYellow()) {
      keeperID = refMsg.yellow().goalkeeper();
    } else {
      keeperID = refMsg.blue().goalkeeper();
    }
    // COLOR DEPENDENT STATES
    if (settings.isYellow()) {
        switch (refMsg.command()) {
            case proto::SSL_Referee_Command_HALT:
                cmd = RefCommand::HALT;
                break;
            case proto::SSL_Referee_Command_STOP:
                cmd = RefCommand::STOP;
                break;
            case proto::SSL_Referee_Command_NORMAL_START:
                cmd = RefCommand::NORMAL_START;
                break;
            case proto::SSL_Referee_Command_FORCE_START:
                cmd = RefCommand::FORCED_START;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                cmd = RefCommand::PREPARE_KICKOFF_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                cmd = RefCommand::PREPARE_KICKOFF_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                cmd = RefCommand::PREPARE_PENALTY_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                cmd = RefCommand::PREPARE_SHOOTOUT_THEM;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW:
                cmd = RefCommand::DIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_BLUE:
                cmd = RefCommand::DIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                cmd = RefCommand::INDIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE:
                cmd = RefCommand::INDIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_YELLOW:
                cmd = RefCommand::TIMEOUT_US;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_BLUE:
                cmd = RefCommand::TIMEOUT_THEM;
                break;
            case proto::SSL_Referee_Command_GOAL_YELLOW:
                cmd = RefCommand::GOAL_US;
                break;
            case proto::SSL_Referee_Command_GOAL_BLUE:
                cmd = RefCommand::GOAL_THEM;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                cmd = RefCommand::BALL_PLACEMENT_US;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                cmd = RefCommand::BALL_PLACEMENT_THEM;
                break;
            default: {
                RTT_ERROR("Unknown refstate, halting all robots for safety!")
                cmd = RefCommand::HALT;
                break;
            }
        }
    } else {
        switch (refMsg.command()) {
            case proto::SSL_Referee_Command_HALT:
                cmd = RefCommand::HALT;
                break;
            case proto::SSL_Referee_Command_STOP:
                cmd = RefCommand::STOP;
                break;
            case proto::SSL_Referee_Command_NORMAL_START:
                cmd = RefCommand::NORMAL_START;
                break;
            case proto::SSL_Referee_Command_FORCE_START:
                cmd = RefCommand::FORCED_START;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:
                cmd = RefCommand::PREPARE_KICKOFF_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE:
                cmd = RefCommand::PREPARE_KICKOFF_US;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
                cmd = RefCommand::PREPARE_PENALTY_THEM;
                break;
            case proto::SSL_Referee_Command_PREPARE_PENALTY_BLUE:
                cmd = RefCommand::PREPARE_SHOOTOUT_US;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_YELLOW:
                cmd = RefCommand::DIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_DIRECT_FREE_BLUE:
                cmd = RefCommand::DIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_YELLOW:
                cmd = RefCommand::INDIRECT_FREE_THEM;
                break;
            case proto::SSL_Referee_Command_INDIRECT_FREE_BLUE:
                cmd = RefCommand::INDIRECT_FREE_US;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_YELLOW:
                cmd = RefCommand::TIMEOUT_THEM;
                break;
            case proto::SSL_Referee_Command_TIMEOUT_BLUE:
                cmd = RefCommand::TIMEOUT_US;
                break;
            case proto::SSL_Referee_Command_GOAL_YELLOW:
                cmd = RefCommand::GOAL_THEM;
                break;
            case proto::SSL_Referee_Command_GOAL_BLUE:
                cmd = RefCommand::GOAL_US;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW:
                cmd = RefCommand::BALL_PLACEMENT_THEM;
                break;
            case proto::SSL_Referee_Command_BALL_PLACEMENT_BLUE:
                cmd = RefCommand::BALL_PLACEMENT_US;
                break;
            default: {
                RTT_ERROR("Unknown refstate, halting all robots for safety!")
                cmd = RefCommand::HALT;
                break;
            }
        }
    }

    auto stage = refMsg.stage();
    auto world = data->getWorld();
    if (world.has_value()) {
        strategymanager.setCurrentRefGameState(cmd, stage, world->getBall());
    }

}

// Initialize static variables
GameState GameStateManager::getCurrentGameState() {
    GameState newGameState;

    bool uses_referee_commands = true; //TODO: make setting/listen to settings
    if (uses_referee_commands) { //TODO: no more static and clean up distinction between strategymanager/interface state
        newGameState = static_cast<GameState>(strategymanager.getCurrentRefGameState());

        newGameState.keeperId = keeperID;

        // if there is a ref we set the interface gamestate to these values as well
        // this makes sure that when we stop using the referee we don't return to an unknown state,
        // // so now we keep the same.
       interface_gamestate=newGameState;
    } else {
        newGameState = interface_gamestate;
    }
    return newGameState;
}

void GameStateManager::forceNewGameState(RefCommand cmd, std::optional<rtt_world::view::BallView> ball) {
    RTT_INFO("Forcing new refstate!")

    // overwrite both the interface and the strategy manager.
    interface_gamestate = strategymanager.getRefGameStateForRefCommand(cmd);

    strategymanager.forceCurrentRefGameState(cmd, ball);
}

bool GameStateManager::canEnterDefenseArea(int robotId) {
    GameState currentState = getCurrentGameState();
    if (robotId != currentState.keeperId) {
        return currentState.getRuleSet().robotsCanEnterDefenseArea();
    }

    return true;
}

bool GameStateManager::canMoveOutsideField(int robotId) {
    GameState currentState = getCurrentGameState();
    if (robotId != currentState.keeperId) {
        return currentState.getRuleSet().robotsCanGoOutOfField;
    }
    return true;
}

Vector2 GameStateManager::getRefereeDesignatedPosition() {
    auto designatedPos = rtt::ai::GameStateManager::getRefereeData().designated_position();
    return Vector2(designatedPos.x() / 1000, designatedPos.x() / 1000);
}
}  // namespace rtt::ai
