//
// Created by timovdk on 3/18/20.
//

#include "stp/new_tactics/Intercept.h"

#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Intercept::Intercept() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
    skills.initialize();
}

bool Intercept::isEndTactic() noexcept {
    // This is an end tactic
    return true;
}

void Intercept::onInitialize() noexcept {}

void Intercept::onUpdate(const Status& status) noexcept {}

void Intercept::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto& x : skills) {
        x->terminate();
    }
}

StpInfo Intercept::calculateInfoForSkill(const StpInfo& info) noexcept {
    StpInfo skillStpInfo = info;

    // Rotate robot towards the ball
    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));

    // If ball is close to robot, turn on dribbler
    skillStpInfo.setDribblerSpeed(determineDribblerSpeed(info.getRobot().value()));

    return skillStpInfo;
}

bool Intercept::isTacticFailing(const StpInfo& info) noexcept {
    // If the ball doesn't move, the robot can't intercept
    return info.getBall()->get()->getVelocity().length() <= stp::control_constants::BALL_STILL_VEL;
}

bool Intercept::shouldTacticReset(const StpInfo& info) noexcept {
    // If the robot does not have the ball, reset so GoToPos is called to move to the ball again
    return !info.getRobot()->hasBall(stp::control_constants::ROBOT_RADIUS);
}

double Intercept::calculateAngle(const world_new::view::RobotView& robot, const world_new::view::BallView& ball) { return (ball->getPos() - robot->getPos()).angle(); }

int Intercept::determineDribblerSpeed(const world_new::view::RobotView& robot) {
    double turnOnDribblerDistance = 1.0;
    return robot->getDistanceToBall() < turnOnDribblerDistance ? 100 : 0;
}
}  // namespace rtt::ai::stp::tactic