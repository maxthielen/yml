#ifndef ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H

#include "Condition.h"

namespace rtt::ai {

class IsRobotClosestToBall : public Condition {
   public:
    IsRobotClosestToBall(std::string name = "IsRobotClosestToBall", bt::Blackboard::Ptr blackboard = nullptr);

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H