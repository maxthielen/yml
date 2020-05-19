//
// Created by jordi on 28-04-20.
//

#include "stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {

uint8_t KickOffThemPrepareGameStateInvariant::metricCheck(world_new::view::WorldDataView, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "kickoff_them_prepare" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}

}  // namespace rtt::ai::stp::invariant