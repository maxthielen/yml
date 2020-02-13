//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?
#include "skills/DribbleRotate.h"
#include "coach/BallplacementCoach.h"
#include "control/ControlUtils.h"
#include "world/FieldComputations.h"

namespace rtt::ai {
DribbleRotate::DribbleRotate(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void DribbleRotate::checkProgression() {
    if (!robot->hasBall()) {
        currentProgression = FAIL;
        return;
    }
    if (abs(robot->angle - targetAngle) < 0.1 * M_PI) {
        currentProgression = SUCCESS;
        return;
    } else {
        currentProgression = ROTATING;
    }
}

void DribbleRotate::onInitialize() {
    if (properties->hasDouble("maxVel")) {
        maxSpeed = properties->getDouble("maxVel");
    } else {
        maxSpeed = MAX_SPEED;
    }
    if (properties->hasDouble("Angle")) {
        targetAngle = Angle(properties->getDouble("Angle"));
    } else if (properties->getBool("RotateToTheirGoal")) {
        Vector2 theirCentre = (*field).getTheirGoalCenter();
        targetAngle = (theirCentre - robot->pos).toAngle();
    } else if (properties->getBool("BallPlacement")) {
        if (properties->getBool("BallPlacementForwards")) {
        }
        targetAngle = (Vector2(robot->pos) - coach::g_ballPlacement.getBallPlacementPos()).toAngle();
    }
    if (!properties->hasDouble("Angle") && !properties->hasBool("RotateToTheirGoal") && !properties->hasBool("BallPlacement")) {
        std::cerr << " dribbleRotate Initialize -> No good angle set in properties" << std::endl;
        currentProgression = FAIL;
    }
    startAngle = robot->angle;
    incrementAngle = maxSpeed / Constants::TICK_RATE();
    currentProgression = ROTATING;
    currentTick = 0;
    extraTick = static_cast<int>(WAIT_TIME * Constants::TICK_RATE());
    dir = Control::rotateDirection(startAngle, targetAngle);
    maxTick = (int)floor(Control::angleDifference(startAngle, targetAngle) / maxSpeed * Constants::TICK_RATE());
    if (!world::world->ourRobotHasBall(robot->id, Constants::MAX_BALL_RANGE())) {
        std::cout << "RobotPtr does not have ball in dribbleRotateInitialize" << std::endl;
        std::cout << "Distance" << (Vector2(robot->pos) - Vector2(ball->getPos())).length() - Constants::ROBOT_RADIUS() << "Max distance:" << Constants::MAX_BALL_RANGE()
                  << std::endl;
        currentProgression = FAIL;
        std::cout << robot->angle.getAngle() << std::endl;
    } else {
        std::cout << "RobotPtr has ball in dribbleRotate Initialize" << std::endl;
    }
}

DribbleRotate::Status DribbleRotate::onUpdate() {
    checkProgression();
    switch (currentProgression) {
        case ROTATING:
            command = ballHandlePosControl.getRobotCommand(world, field, robot, ball->getPos(), targetAngle).makeROSCommand();
            publishRobotCommand();
            return Status::Running;
        case SUCCESS:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }
    return Status::Failure;
}

void DribbleRotate::onTerminate(Status s) {
    command.set_dribbler(31);
    command.set_w(static_cast<float>(targetAngle));
    currentProgression = ROTATING;
    currentTick = 0;
    publishRobotCommand();
}

void DribbleRotate::sendMoveCommand() {
    command.set_dribbler(31);
    command.set_w(static_cast<float>(computeCommandAngle()));
    currentTick++;
    publishRobotCommand();
}

double DribbleRotate::computeCommandAngle() {
    if (currentTick < maxTick) {
        return Control::constrainAngle(startAngle + dir * currentTick * incrementAngle);
    }
    return targetAngle;
}

}  // namespace rtt::ai