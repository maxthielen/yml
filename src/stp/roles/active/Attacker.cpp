//
// Created by jordi on 17-03-20.
/// TODO-Max change to ShooterGoal
//

#include "include/roboteam_ai/stp/roles/active/Attacker.h"

#include "include/roboteam_ai/stp/tactics/active/GetBall.h"
#include "include/roboteam_ai/stp/tactics/active/KickAtPos.h"

namespace rtt::ai::stp::role {

Attacker::Attacker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos()};
}
}  // namespace rtt::ai::stp::role
