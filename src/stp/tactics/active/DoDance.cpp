//
// Created by Jesse on 23-06-20.

/// ACTIVE



#include "stp/tactics/active/DoDance.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Circle.h"


namespace rtt::ai::stp::tactic {

DoDance::DoDance() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Circle()};
}

std::optional<StpInfo> DoDance::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    return skillStpInfo;
}


bool DoDance::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool DoDance::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    return false;
}

bool DoDance::shouldTacticReset(const StpInfo &info) noexcept {
    return false;
}

const char *DoDance::getName() { return "Dance"; }

}  // namespace rtt::ai::stp::tactic
