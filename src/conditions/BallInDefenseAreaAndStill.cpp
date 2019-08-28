/*
 * returns SUCCESS if the ball is in the given defence area (standard ours) 
 * AND if the ball lays still
 */

#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/world/Ball.h>
#include "include/roboteam_ai/conditions/BallInDefenseAreaAndStill.h"
#include "include/roboteam_ai/utilities/Constants.h"

namespace rtt {
namespace ai {

BallInDefenseAreaAndStill::BallInDefenseAreaAndStill(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void BallInDefenseAreaAndStill::onInitialize() {
    theirDefenceArea = properties->getBool("theirDefenceArea");
    outsideField = properties->getBool("outsideField");
}

bt::Node::Status BallInDefenseAreaAndStill::onUpdate() {
    Vector2 ballPos = ball->pos;
    Vector2 ballVel=ball->vel;

    bool pointIsInDefenceArea = world::field->pointIsInDefenceArea(ballPos, !theirDefenceArea,0.02,false);
    bool ballIsLayingStill = ballVel.length() < Constants::BALL_STILL_VEL();
    if (pointIsInDefenceArea && ballIsLayingStill){
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
