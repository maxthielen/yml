//
// Created by robzelluf on 3/21/19.
//

#include "skills/DemoAttack.h"
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include <control/PositionUtils.h>
#include <world/FieldComputations.h>

namespace rtt::ai {

DemoAttack::DemoAttack(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void DemoAttack::onInitialize() {
    robot->getNumtreePosControl()->setAvoidBallDistance(Constants::DEFAULT_BALLCOLLISION_RADIUS());
    ownGoal = properties->getBool("ownGoal");
    shot = false;
}

/// Get an update on the skill
bt::Node::Status DemoAttack::onUpdate() {
    if (!robot) return Status::Running;

    if (shot && !world->robotHasBall(robot->id, true)) {
        return Status::Success;
    }

    Vector2 ball = world->getBall()->getPos();
    Vector2 behindBall = control::PositionUtils::getPositionBehindBallToGoal(*field, BEHIND_BALL_TARGET, ownGoal);
    Vector2 deltaBall = behindBall - ball;

    if (!control::PositionUtils::isRobotBehindBallToGoal(*field, BEHIND_BALL_CHECK, ownGoal, robot->pos)) {
        targetPos = behindBall;
        command.set_w(static_cast<float>((ball - (Vector2)(robot->pos)).angle()));
        robot->getNumtreePosControl()->setAvoidBallDistance(Constants::DEFAULT_BALLCOLLISION_RADIUS());

        if (abs(((Vector2)robot->pos - targetPos).length()) < SWITCH_TO_BASICGTP_DISTANCE) {
            robot->getNumtreePosControl()->setAvoidBallDistance(0);
        }
    } else {
        targetPos = ball;
        robot->getNumtreePosControl()->setAvoidBallDistance(0);

        command.set_w(static_cast<float>(((Vector2){-1.0, -1.0} * deltaBall).angle()));
        if (world::world->robotHasBall(robot->id, true)) {
            command.set_kicker(true);
            command.set_chip_kick_vel(static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER()));
            command.set_chip_kick_forced(true);
            shot = true;
        }
    }
    Vector2 velocity;
    if (FieldComputations::pointIsInDefenceArea(*field, robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2)robot->pos - (*field).getOurGoalCenter()).stretchToLength(2.0);
    } else if (FieldComputations::pointIsInDefenceArea(*field, robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2)robot->pos - (*field).getTheirGoalCenter()).stretchToLength(2.0);
    } else if (FieldComputations::pointIsInDefenceArea(*field, ball, ownGoal) || FieldComputations::pointIsInDefenceArea(*field, ball, !ownGoal)) {
        velocity = {0, 0};
    } else if (FieldComputations::pointIsInDefenceArea(*field, targetPos, ownGoal)) {
        velocity = {0, 0};
    } else {
        velocity = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, targetPos).vel;
    }

    command.mutable_vel()->set_x(static_cast<float>(velocity.x));
    command.mutable_vel()->set_y(static_cast<float>(velocity.y));
    publishRobotCommand();

    return Status::Running;
}

void DemoAttack::onTerminate(Status s) {
    command.set_w(static_cast<float>(deltaPos.angle()));
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    publishRobotCommand();
}

}  // namespace rtt::ai