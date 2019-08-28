#include <include/roboteam_ai/control/PositionUtils.h>
#include "include/roboteam_ai/skills/KickTo.h"
#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/control/numTrees/NumTreePosControl.h>
#include <include/roboteam_ai/control/BasicPosControl.h>
#include <include/roboteam_ai/control/ControlUtils.h>

namespace rtt {
namespace ai {

KickTo::KickTo(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}
void KickTo::onInitialize() {
    std::string type=properties->getString("type");
    if (type=="shootout"){
        shootPos=Vector2(world::field->get_field().field_length*0.2,0); // 2.4 m for A field, 1.8 for B
    }
    else{
        shootPos=Vector2(0,0);
    }
}
/// Get an update on the skill
bt::Node::Status KickTo::onUpdate() {
    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.w = 0;
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = shootPos;
    //TODO: tune kick velocity
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::BALL_PLACEMENT, true, control::ShotPrecision::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt