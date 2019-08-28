//
// Created by baris on 10-5-19.
//

#include <include/roboteam_ai/interface/api/Output.h>
#include "include/roboteam_ai/skills/DriveWithInterface.h"

namespace rtt {
namespace ai {
DriveWithInterface::DriveWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
Skill::Status DriveWithInterface::onUpdate() {


    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }
    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto robotCommand = numTreeGtp.getRobotCommand(robot, targetPos);

    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);
    publishRobotCommand();
    return Status::Running;
}
}
}