//
// Created by jordi on 19-05-20.
//

#include "stp/roles/BallReflector.h"

#include "stp/tactics/KickAtPos.h"
#include "stp/tactics/PositionAndAim.h"

namespace rtt::ai::stp::role {

BallReflector::BallReflector(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::PositionAndAim(), tactic::KickAtPos()};
}
}  // namespace rtt::ai::stp::role