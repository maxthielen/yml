
//
// Created by thijs on 25-5-19.
//

#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/interface/api/Input.h>
#include "include/roboteam_ai/control/ControlUtils.h"
#include "include/roboteam_ai/control/ballHandling/DribbleBackwards.h"
#include "include/roboteam_ai/control/ballHandling/RotateAroundBall.h"
#include "include/roboteam_ai/control/ballHandling/RotateWithBall.h"
#include "include/roboteam_ai/world/Ball.h"
#include "include/roboteam_ai/world/World.h"
#include <iostream>
#include <sstream>

namespace rtt {
namespace ai {
namespace control {

RobotCommand DribbleBackwards::getRobotCommand(RobotPtr r, const Vector2 &targetP, const Angle &targetA) {

    robot = std::move(r);
    ball = world::world->getBall();
    finalTargetAngle = targetA;
    targetAngle = targetA;
    finalTargetPos = targetP;
    targetPos = targetP;

    updateBackwardsProgress();
    return sendBackwardsCommand();
}

void DribbleBackwards::reset() {
    backwardsProgress = START;
}

void DribbleBackwards::updateBackwardsProgress() {
    if (Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO()) {
        printBackwardsProgress();
    }

    if (backwardsProgress != DRIBBLING && backwardsProgress != OVERSHOOTING) waitingTicks = 100;

    // check if we still have ball
    if (backwardsProgress != OVERSHOOTING &&
            backwardsProgress != DRIBBLING &&
            backwardsProgress != DRIBBLE_BACKWARDS) {

        approachPosition = ball->pos + (robot->pos - ball->pos).stretchToLength(-0.05);
    }
    if ((ball->pos - robot->pos).length2() > 0.5) {
        backwardsProgress = START;
        return;
    }
    targetAngle = (ball->pos - finalTargetPos).toAngle();
    Angle angleDifference = robot->angle - targetAngle;

    // update backwards progress
    switch (backwardsProgress) {
    case TURNING: {
        targetAngle = (ball->pos - finalTargetPos).toAngle();
        if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
            lockedAngle = targetAngle;
            backwardsProgress = APPROACHING;
        }
        return;
    }
    case APPROACHING: {
        if (fabs(angleDifference) > angleErrorMargin) {
            backwardsProgress = TURNING;
            return;
        }
        if (robot->hasBall()) {
            backwardsProgress = OVERSHOOTING;
            return;
        }
        return;
    }
    case OVERSHOOTING: {
        if (! robot->hasBall()) {
            backwardsProgress = START;
            return;
        }
        double overshoot = 0.06;
        if (((approachPosition - robot->pos)).length() < overshoot) {
            backwardsProgress = DRIBBLING;
            return;
        }
        if (-- waitingTicks < 0) {
            failedOnce = true;
            backwardsProgress = DRIBBLING;
            return;
        }
        return;
    }
    case DRIBBLING: {
        if (! robot->hasBall()) {
            backwardsProgress = APPROACHING;
            return;
        }
        if (-- waitingTicks < 75) {
            backwardsDribbleLine = {robot->pos, finalTargetPos};
            backwardsProgress = DRIBBLE_BACKWARDS;
            return;
        }
        return;
    }
    case DRIBBLE_BACKWARDS: {
        if (! robot->hasBall()) {

            backwardsProgress = APPROACHING;
            return;
        }
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            backwardsProgress = SUCCESS;
            return;
        }
        return;
    }
    case FAIL:
    case START: {
        failedOnce = false;
    }
    default:return;
    case SUCCESS: {
        if ((ball->pos - finalTargetPos).length2() < ballPlacementAccuracy*ballPlacementAccuracy) {
            return;
        }
        backwardsProgress = START;
    }
    }
}

RobotCommand DribbleBackwards::sendBackwardsCommand() {
    switch (backwardsProgress) {
    case START: return startTravelBackwards();
    case TURNING:return sendTurnCommand();
    case APPROACHING:return sendApproachCommand();
    case OVERSHOOTING:return sendOvershootCommand();
    case DRIBBLING:return sendDribblingCommand();
    case DRIBBLE_BACKWARDS:return sendDribbleBackwardsCommand();
    case SUCCESS:return sendSuccessCommand();
    case FAIL: {
        backwardsProgress = START;
        return {};
    }
    default: return {};
    }
}

RobotCommand DribbleBackwards::startTravelBackwards() {
    approachPosition = Vector2();
    lockedAngle = Angle();
    backwardsDribbleLine = {};
    backwardsProgress = TURNING;
    return sendTurnCommand();
}

RobotCommand DribbleBackwards::sendTurnCommand() {
    if (fabs(targetAngle - robot->angle) < angleErrorMargin) {
        lockedAngle = targetAngle;
    }
    targetPos = finalTargetPos;
    return rotateAroundBall->getRobotCommand(robot, targetPos, targetAngle);
}

RobotCommand DribbleBackwards::sendApproachCommand() {
    RobotCommand command;
    command.dribbler = 31;
    command.vel = (ball->pos - robot->pos).stretchToLength(std::min(0.2, maxVel));
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleBackwards::sendOvershootCommand() {
    RobotCommand command;
    command.dribbler = 31;
    command.vel = (approachPosition - robot->pos).stretchToLength(std::min(0.2, maxVel));
    command.angle = lockedAngle;

    if (failedOnce && !world::field->pointIsInField(ball->pos, Constants::ROBOT_RADIUS())) {
        command.kickerForced = true;
        command.kickerVel = 2;
        command.kicker = true;
    }
    return command;
}

RobotCommand DribbleBackwards::sendDribblingCommand() {
    RobotCommand command;
    command.dribbler = 31;
    command.angle = lockedAngle;
    return command;
}

RobotCommand DribbleBackwards::sendDribbleBackwardsCommand() {
    RobotCommand command;
    command.dribbler = 31;
    command.angle = lockedAngle;
    command.vel = lockedAngle.toVector2(- maxVel);


    // check if the robot is still on the virtual line from ball->pos to the target
    if (control::ControlUtils::distanceToLine(robot->pos,
            backwardsDribbleLine.first, backwardsDribbleLine.second) > errorMargin*5) {
        backwardsProgress = TURNING;
    }

    Angle robotAngleTowardsLine = (finalTargetPos - robot->pos).toAngle() -
            (backwardsDribbleLine.second - backwardsDribbleLine.first).toAngle();

    Vector2 compensation = (robot->angle + M_PI_2).toVector2(std::min(robotAngleTowardsLine*2.72, 0.05));
    command.vel += compensation;

    interface::Input::drawData(interface::Visual::BALL_HANDLING, {compensation+robot->pos, robot->pos},
            Qt::white, robot->id, interface::Drawing::ARROWS);
    interface::Input::drawData(interface::Visual::BALL_HANDLING, {backwardsDribbleLine.first, backwardsDribbleLine.second},
            Qt::white, robot->id, interface::Drawing::LINES_CONNECTED);

    // check if the ball is not too far right or too far left of the robot, and try to compensate for that
    if (ball->visible && false) {
        Angle ballAngleRelativeToRobot = (ball->pos - robot->pos).toAngle() - robot->angle;
        command.vel += (robot->angle + M_PI_2).toVector2(ballAngleRelativeToRobot);
    }

    return command;
}

RobotCommand DribbleBackwards::sendSuccessCommand() {
    RobotCommand command;
    command.dribbler = 0;
    command.angle = lockedAngle;
    return command;
}

void DribbleBackwards::printBackwardsProgress() {
    std::stringstream ss;
    ss << "backwards progress:                  ";
    switch (backwardsProgress) {
    case OVERSHOOTING:ss << "overshooting";
        break;
    case DRIBBLING:ss << "dribbling";
        break;
    case DRIBBLE_BACKWARDS:ss << "dribbleBackwards";
        break;
    case SUCCESS:ss << "success";
        break;
    case FAIL:ss << "fail";
        break;
    case START:ss << "start";
        break;
    case TURNING:ss << "turning";
        break;
    case APPROACHING:ss << "approaching";
        break;
    }
    std::cout << ss.str() << std::endl;
}

DribbleBackwards::BackwardsProgress DribbleBackwards::getBackwardsProgression() {
    return backwardsProgress;
}

DribbleBackwards::DribbleBackwards(double errorMargin, double angularErrorMargin, double ballPlacementAccuracy, double maxVel)
        :waitingTicks(0), errorMargin(errorMargin), angleErrorMargin(angularErrorMargin),
         ballPlacementAccuracy(ballPlacementAccuracy), maxVel(maxVel) {

    rotateAroundBall = new RotateAroundBall();
    rotateAroundRobot = new RotateWithBall();
}

DribbleBackwards::~DribbleBackwards() {
    delete rotateAroundBall;
    delete rotateAroundRobot;
}
void DribbleBackwards::setMaxVel(double maxV) {
    maxVel = maxV;
}

}
}
}
