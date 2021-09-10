//
// Created by jordi on 09-03-20.
//

#include "stp/skills/Circle.h"

#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status Circle::onUpdate(const StpInfo &info) noexcept {
    Vector2 startPos = info.getPositionToMoveTo().value();
    Vector2 curPos = info.getRobot().value()->getPos();

    Vector2 posDiff = startPos - curPos;

    rtt::BB::CommandCollision commandCollision;

    if (commandCollision.collisionData.has_value()) {
        return Status::Failure;
    }

    Vector2 targetVelocity = Vector2(10 * sin(posDiff.y),10 * -sin(posDiff.x));

    // Set velocity and angle commands
    command.mutable_vel()->set_x(static_cast<float>(targetVelocity.x));
    command.mutable_vel()->set_y(static_cast<float>(targetVelocity.y));

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    RTT_DEBUG("Running Circle")
    if (targetVelocity.x < -0.1 && targetVelocity.y > 0){
        RTT_DEBUG("Made full circle!")
        RTT_DEBUG(posDiff.length())
        RTT_DEBUG(targetVelocity)
        command.mutable_vel()->set_x(static_cast<float>(0));
        command.mutable_vel()->set_y(static_cast<float>(0));
        return Status::Success;
    }
    return Status::Running;
}

const char *Circle::getName() { return "Circle"; }

}  // namespace rtt::ai::stp::skill