//
// Created by robzelluf on 4/25/19.
//

#include <coach/PassCoach.h>
#include "Condition.h"

#ifndef ROBOTEAM_AI_ISPASSHAPPENING_H
#define ROBOTEAM_AI_ISPASSHAPPENING_H

namespace rtt::ai {

class IsOnPassLine : public Condition {
   private:
    const double DISTANCE_FROM_PASS_LINE = 5 * Constants::ROBOT_RADIUS();

   public:
    explicit IsOnPassLine(std::string name = "IsOnPassLine", bt::Blackboard::Ptr blackboard = nullptr);

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ISPASSHAPPENING_H