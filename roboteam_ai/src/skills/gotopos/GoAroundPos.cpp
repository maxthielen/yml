//
// Created by rolf on 30-1-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "GoAroundPos.h"
#include "roboteam_ai/src/interface/drawer.h"

namespace rtt {
namespace ai {

GoAroundPos::GoAroundPos(rtt::string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(name, blackboard) { }

void GoAroundPos::gtpInitialize() {
    if (properties->hasBool("ball")) {
        if (ball) {
            ballIsTarget = true;
            targetPos = ball->pos;
        }
        else {
            ROS_ERROR("Get some balls");
        }
    }
    else {
        ballIsTarget = false;
        targetPos = properties->getVector2("targetPos");
    }

    if (properties->hasDouble("targetDir")) {
        endAngle = Control::constrainAngle(properties->getDouble("targetDir"));
    }
    else if (properties->getBool("towardsTheirGoal")) {
        endAngle = Control::constrainAngle((world::field->get_their_goal_center() - targetPos).angle());
    }
    else {
        endAngle = 0;
        ROS_ERROR_STREAM("GoAroundPos update --> No target direction set! Defaulting to 0");
    }

    deltaPos = targetPos - robot->pos;
    startAngle = deltaPos.angle();
    rotateDir = Control::rotateDirection(startAngle, endAngle);
    if (ballIsTarget) {
        distanceFromPoint = BALL_DIST;
    }
    else {
        if (properties->hasDouble("RotatingDistance")) {
            distanceFromPoint = properties->getDouble("RotatingDistance");
        }
        else {
            ROS_ERROR_STREAM("No rotating distance set! Defaulting to ball distance");
            distanceFromPoint = BALL_DIST;
        }
    }
    currentTick = 0;
    angleDif = Control::angleDifference(startAngle, endAngle);
    maxTick = floor(angleDif/SPEED*Constants::TICK_RATE());
    currentProgress = ROTATING;
}

GoAroundPos::Status GoAroundPos::gtpUpdate() {
    if (! robot) {
        ROS_ERROR("Robot not found ree:  %s", std::to_string(robot->id).c_str());
        return Status::Failure;
    }
    if (ballIsTarget && ! ball) {
        ROS_ERROR_STREAM("GoAroundPos update -> No ball found!");
        return Status::Failure;
    }
    if (ballIsTarget) {
        targetPos = ball->pos;
    }
    if (currentTick <= maxTick) {
        commandPos = targetPos + Vector2(distanceFromPoint, 0).rotate(
                startAngle + rotateDir*currentTick/maxTick*angleDif + M_PI);
    }
    else {
        commandPos = targetPos + Vector2(distanceFromPoint, 0).rotate(endAngle + M_PI);
    }

    deltaPos = targetPos - robot->pos;
    currentProgress = checkProgression();
    currentTick ++;

    // Visualization
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    displayColorData.emplace_back(commandPos, Qt::red);
    displayColorData.emplace_back(targetPos + Vector2(distanceFromPoint, 0).rotate(endAngle + M_PI), Qt::red);

    interface::Drawer::setNumTreePoints(robot->id, displayColorData);

    switch (currentProgress) {
    case ROTATING: {
        sendRotateCommand();
        return Status::Running;
    }
    case STOPPING: {
        sendRotateCommand();
        return Status::Running;
    }
    case FAIL: return Status::Failure;
    case DONE: return Status::Success;
    }
    return Status::Failure;
}

void GoAroundPos::gtpTerminate(rtt::ai::Skill::Status s) {
    command.w = (float) deltaPos.angle();
}

GoAroundPos::Progression GoAroundPos::checkProgression() {
    //Failure condition: If it goes outside of the margin during any phase but stopping (also helps against ball moving etc.)
    if (currentProgress != STOPPING) {
        if (! checkPosition()) {
            checkPosition();
            return FAIL;
        }
    }

    //Go to stopping if we are done rotating
    if (currentProgress == ROTATING) {
        if (currentTick > maxTick) {
            return STOPPING;
        }
        else return ROTATING;
    }

    if (currentProgress == STOPPING) {
        //Done when robot sufficiently close to desired end position and rotation.
        double angDif = Control::angleDifference(deltaPos.angle(), endAngle);
        double posDif = (commandPos - robot->pos).length();
        if (posDif < POS_MARGIN && angDif < ANGLE_MARGIN) {
            return DONE;
        }
        //If Robot takes too long to stop, fail
        if (currentTick > maxTick + MAX_STOP_TIME*Constants::TICK_RATE()) {
            return FAIL;
        }
        else return STOPPING;
    }
    return FAIL;
}

bool GoAroundPos::checkPosition() {
    double currentAngle = deltaPos.angle();
    double totalSum =
            Control::angleDifference(startAngle, currentAngle) + Control::angleDifference(currentAngle, endAngle);
    if (totalSum > angleDif + 0.1*M_PI*2) {
        return false;
    }
    return ((deltaPos.length() <= (distanceFromPoint + MAX_DIST_DEVIATION))
            && deltaPos.length() > distanceFromPoint - MAX_DIST_DEVIATION);
}

void GoAroundPos::sendRotateCommand() {
    Vector2 deltaCommandPos = (commandPos - robot->pos);
    deltaCommandPos = Control::velocityLimiter(deltaCommandPos, distanceFromPoint*SPEED, MIN_SPEED);
    command.dribbler = 0;
    command.x_vel = static_cast<float>(deltaCommandPos.x);
    command.y_vel = static_cast<float>(deltaCommandPos.y);
    command.w = (float) deltaPos.angle();
    publishRobotCommand();
}
}
}