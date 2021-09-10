//
// Created by jordi on 17-03-20.
/// TODO-Max change to ShooterGoal
//

#include "stp/roles/active/Dancer.h"

#include "stp/tactics/active/DoDance.h"
#include "stp/tactics/passive/Formation.h"



namespace rtt::ai::stp::role {

Dancer::Dancer(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::DoDance(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
