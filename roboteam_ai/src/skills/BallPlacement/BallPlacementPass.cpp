//
// Created by mrlukasbos on 3-5-19.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "BallPlacementPass.h"

namespace rtt {
namespace ai {


BallPlacementPass::BallPlacementPass(string name, bt::Blackboard::Ptr blackboard)
    : Pass(name, blackboard) { }


void BallPlacementPass::onInitialize() {
    robotToPassToID = -1;
    hasShot = false;
    shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::HIGH, control::BallSpeed::LAY_STILL_AT_POSITION, true);
}

bt::Node::Status BallPlacementPass::onUpdate() {

    targetPos = coach::g_ballPlacement.getBallPlacementPos();


    robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
    if (robotToPassToID == -1) {
        std::cout << "the robot to pass to id is -1" << std::endl;
        publishRobotCommand(); //halt

        return Status::Failure;
    }

    robotToPassTo = world::world->getRobotForId(robotToPassToID, true);
//    if(!coach::g_pass.validReceiver(robot, robotToPassTo)) {
//        std::cout << "the receiver is invalid" << std::endl;
//        publishRobotCommand(); // halt
//        return Status::Failure;
//    }

    if (ball->pos.dist(targetPos) < 0.5) {
        publishRobotCommand();
        return Status::Running;
    }

    if (didShootProperly()) {
        hasShot = true;
        coach::g_pass.setPassed(true);
        publishRobotCommand();
        return Status::Success;
    }

    /*
     * Make the shot if the receiver is ready
     * Otherwise we can already drive to the position but wait while close
     * When receiver is ready we can shoot
     */
    if (!coach::g_pass.isPassed() && !hasShot) {
        if (coach::g_pass.isReadyToReceivePass()) {
            shotControl->makeCommand(shotControl->getShotData(*robot, getKicker(), false), command);
        } else if (robot->pos.dist(ball->pos) > 0.5) {
            auto pva = numTreeGtp.getPosVelAngle(robot, ball->pos);
            command.x_vel = pva.vel.x;
            command.y_vel = pva.vel.y;
            command.w = pva.angle;
            // empty command
        } else {
            command.w = (ball->pos - robot->pos).angle();
        }
    }


    publishRobotCommand();
    return Status::Running;
}


}
}