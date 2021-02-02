//
// Created by Max on 01-01-2021
//

#include "stp/roles/DMGhosting.h"

#include "stp/tactics/DMGoThrough.h"
#include "stp/tactics/Intercept.h"

namespace rtt::ai::stp::role {

DMGhosting::DMGhosting(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::DMGoThrough()};
}
}  // namespace rtt::ai::stp::role
