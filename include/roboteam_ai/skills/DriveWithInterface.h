//
// Created by baris on 10-5-19.
//

#ifndef ROBOTEAM_AI_DRIVEWITHINTERFACE_H
#define ROBOTEAM_AI_DRIVEWITHINTERFACE_H

#include <random>
#include "roboteam_utils/Vector2.h"
#include "Skill.h"
#include "control/positionControl/PositionControl.h"

namespace rtt::ai {

class DriveWithInterface : public Skill {

public:
    std::mt19937 mt;
    std::uniform_real_distribution<double> randDistribution;

    std::vector<Vector2> path = std::vector<Vector2>();
    std::vector<Vector2> staticPath = std::vector<Vector2>();
    int iStaticPath = 0;


    explicit DriveWithInterface(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIVEWITHINTERFACE_H
