//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <algorithm>
#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {


class OffensiveCoach {
public:
    const double ATTACKER_DISTANCE = 1.6;
    const double OFFENSIVE_POSITION_DISTANCE = 0.8;

    struct OffensivePosition {
    Vector2 position;
    double score;
    };

    void calculateNewPositions();
    void calculateNewRobotPositions(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
    Vector2 calculatePositionForRobot(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
    void releaseRobot(int robotID);
    Vector2 getPositionForRobotID(int robotID);
    std::vector<OffensivePosition> getRobotPositionVectors();
    int getBestStrikerID();
    const vector<OffensivePosition> &getOffensivePositions();
    const map<int, OffensivePosition> &getRobotPositions();


private:

    double marginFromLines = 0.2;

    std::vector<OffensivePosition> offensivePositions;
    int maxPositions = 4;
    std::map<int, OffensivePosition> robotPositions;

    static bool compareByScore(OffensivePosition position1, OffensivePosition position2);
    void drawOffensivePoints();
    void recalculateOffensivePositions();
    OffensivePosition calculateRandomPosition(double xStart, double xEnd, double yStart, double yEnd);
    bool positionTooCloseToRobotPositions(OffensivePosition position, int self = -1);

    void compareToCurrentPositions(const OffensivePosition &position);

    Vector2 getClosestOffensivePosition(const shared_ptr<roboteam_msgs::WorldRobot> &robot);
};

extern OffensiveCoach g_offensiveCoach;

}
}
}


#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
