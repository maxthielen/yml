//
// Created by thijs on 15-5-19.
//

#include "BallPlacementWithInterface.h"

//
// Created by baris on 10-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "DriveWithInterface.h"

namespace rtt {
namespace ai {
BallPlacementWithInterface::BallPlacementWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

Skill::Status BallPlacementWithInterface::onUpdate() {

    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto rc = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle);
    Vector2 velocity = control::ControlUtils::velocityLimiter(rc.vel, Constants::MAX_VEL());
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = rc.angle;
    command.dribbler = rc.dribbler;
    publishRobotCommand();
    return Status::Running;
}

}
}