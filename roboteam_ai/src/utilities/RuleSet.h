//
// Created by mrlukasbos on 11-5-19.
//

#ifndef ROBOTEAM_AI_RULESET_H
#define ROBOTEAM_AI_RULESET_H

namespace rtt {
namespace ai {

struct RuleSet {
    // rules for this specific refgamestate
    std::string title;
    double maxRobotVel;
    double maxCollisionVel;
    double maxBallVel;
    double minDistanceToBall;
    bool robotsCanEnterDefenseArea;
    bool robotsCanGoOutOfField;
};
}
}

#endif //ROBOTEAM_AI_RULESET_H