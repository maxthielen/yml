//
// Created by mrlukasbos on 6-12-18.
//

#include "include/roboteam_ai/skills/Halt.h"

namespace rtt {
namespace ai {

Halt::Halt(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

Halt::Status Halt::onUpdate() {
    // send slowing down command
    command.set_geneva_state(3);
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt