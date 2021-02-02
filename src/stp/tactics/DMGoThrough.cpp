//
// Created by Max on 01-01-2021
//

#include "stp/tactics/DMGoThrough.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

DMGoThrough::DMGoThrough() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> DMGoThrough::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    auto desiredRobotPosition = calculateDesiredRobotPosition(info.getEnemyRobot().value());
    skillStpInfo.setPositionToMoveTo(desiredRobotPosition);

    return skillStpInfo;
}

Vector2 DMGoThrough::calculateDesiredRobotPosition(const world::view::RobotView enemy) {
    return enemy->getPos();
}

bool DMGoThrough::isEndTactic() noexcept { return true; }

bool DMGoThrough::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool DMGoThrough::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *DMGoThrough::getName() { return "DM Go Through"; }

}  // namespace rtt::ai::stp::tactic
