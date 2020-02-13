//
// Created by baris on 23-4-19.
//

#ifndef ROBOTEAM_AI_FREEKICKFORMATION_H
#define ROBOTEAM_AI_FREEKICKFORMATION_H

#include "Formation.h"
namespace rtt::ai {

class FreeKickFormation : public Formation {
   public:
    explicit FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::shared_ptr<std::vector<RobotPtr>> robotsInFormationPtr() override;
    static std::shared_ptr<std::vector<RobotPtr>> robotsInFormation;
    static std::vector<Vector2> posses;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_FREEKICKFORMATION_H