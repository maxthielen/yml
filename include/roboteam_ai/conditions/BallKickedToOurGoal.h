#ifndef ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
#define ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H

#include "Condition.h"

namespace rtt::ai {

class BallKickedToOurGoal : public Condition {
   private:
    const double BALL_TO_GOAL_MARGIN = 0.12;  // m
    const double BALL_TO_GOAL_TIME = 2.5;     // s
   public:
    explicit BallKickedToOurGoal(std::string name = "BallKickedToOurGoal", bt::Blackboard::Ptr blackboard = nullptr);

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H