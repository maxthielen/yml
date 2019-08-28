#include <include/roboteam_ai/analysis/DecisionMaker.h>
#include <include/roboteam_ai/analysis/GameAnalyzer.h>
#include <include/roboteam_ai/world/Field.h>
#include "include/roboteam_ai/skills/formations/KickOffUsFormation.h"
#include "include/roboteam_ai/control/ControlUtils.h"
#include "include/roboteam_ai/control/Hungarian.h"

namespace rtt {
namespace ai {

    std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> KickOffUsFormation::robotsInFormation = nullptr;

    KickOffUsFormation::KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
        robotsInFormation = std::make_shared<std::vector<bt::Leaf::RobotPtr>>();
    }

Vector2 KickOffUsFormation::getFormationPosition() {
    std::vector<int> robotIds;
    auto field = world::field->get_field();
    double fh = field.field_width();
    double fw = field.field_length();

    std::vector<std::vector<Vector2>> locations = {
            {{-0.2,0}},
            {{-0.2,0}, {-0.2, -fh/3}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}, {-fw/6, 0}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}, {-fw/6, -fh/4}, {-fw/7,  fh/4}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}, {-fw/3, 0}},
            {{-0.2,0}, {-0.2, -fh/3}, {-0.2,  fh/3}, {-fw/6, -fh/4}, {-fw/6,  fh/4}, {-fw/7,  0}, {-fw/3, -fh/6}, {-fw/3, fh/6}}
    };

    for (auto const &robot : * robotsInFormation) {
        if (robot) {
            robotIds.push_back(robot->id);
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, locations[robotsInFormation->size()-1]);
    return shortestDistances.at(robot->id);
}

std::shared_ptr<std::vector<bt::Leaf::RobotPtr>> KickOffUsFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

} // ai
} // rtt