//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/interface/widget.h>
#include <roboteam_ai/src/interface/drawer.h>
#include "OffensiveCoach.h"

namespace rtt {
namespace ai {
namespace coach {

OffensiveCoach g_offensiveCoach;

/// Calculate new positions close to the robot
OffensiveCoach::OffensivePosition OffensiveCoach::calculateNewRobotPosition(const OffensivePosition& currentPosition, const Vector2& defaultPosition) {

    OffensivePosition bestPosition = currentPosition;
    bestPosition.score = CoachHeuristics::calculatePositionScore(bestPosition.position);

    for (int xDiff = -GRID_SIZE; xDiff <= GRID_SIZE; xDiff++) {
        if (currentPosition.position.x < 0 && xDiff <= 0) continue;

        for (int yDiff = -GRID_SIZE; yDiff <= GRID_SIZE; yDiff++) {
            OffensivePosition newPosition;
            newPosition.position.x = currentPosition.position.x + SEARCH_GRID_ROBOT_POSITIONS * xDiff;
            newPosition.position.y = currentPosition.position.y + SEARCH_GRID_ROBOT_POSITIONS * yDiff;

            if (!Field::pointIsInField(newPosition.position, 0.10)
            || Field::pointIsInDefenceArea(newPosition.position, false)){
                continue;
            }

            if ((newPosition.position - defaultPosition).length() > ZONE_RADIUS) {
                continue;
            }

            bool tooCloseToOtherZone = false;
            for (auto& otherDefaultPosition : getDefaultLocations()) {
                if (otherDefaultPosition != defaultPosition) {
                    if ((otherDefaultPosition - newPosition.position).length() < (defaultPosition - newPosition.position).length()) {
                        tooCloseToOtherZone = true;
                        break;
                    }
                }
            }
            if (tooCloseToOtherZone) continue;

            newPosition.score = CoachHeuristics::calculatePositionScore(newPosition.position);
            if (newPosition.score > bestPosition.score) {
                bestPosition = newPosition;
            }
        }
    }

    return bestPosition;
}

/// Set offensive positions to be drawn
void OffensiveCoach::drawOffensivePoints() {
    /// Draw general offensive points
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    for (auto &offensivePosition : offensivePositions) {
        displayColorData.emplace_back(std::make_pair(offensivePosition.position, Qt::green));
    }
    interface::Drawer::setOffensivePoints(displayColorData);

    /// Draw attacker points specific to robot
    displayColorData = {};
    for (const auto& robotPosition : robotPositions) {
        displayColorData.emplace_back(std::make_pair(robotPosition.second.position, Qt::darkYellow));
        interface::Drawer::setAttackerPoints(robotPosition.first, displayColorData);
    }
}

/// Get robot with highest offensive score
int OffensiveCoach::getBestStrikerID() {
    double bestScore = 0;
    int bestStriker = -1;
    for(auto& robot : World::get_world().us) {
        if (robot.pos.x > 0) {
            double positionScore = CoachHeuristics::calculatePassScore(robot.pos);
            if (positionScore > bestScore) {
                bestScore = positionScore;
                bestStriker = robot.id;
            }
        }
    }
    return bestStriker;
}

std::vector<Vector2> OffensiveCoach::getDefaultLocations() {
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    Vector2 penaltyStretchCorner = field.top_right_penalty_stretch.end;
    penaltyStretchCorner.x = abs(penaltyStretchCorner.x);
    penaltyStretchCorner.y = abs(penaltyStretchCorner.y);

    std::vector<Vector2> defaultPositions;

    // Calculate two positions close to goal
    defaultPositions.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, penaltyStretchCorner.y + CLOSE_TO_GOAL_DISTANCE);
    defaultPositions.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, -penaltyStretchCorner.y - CLOSE_TO_GOAL_DISTANCE);

    // Calculate two positions further from goal
    defaultPositions.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, penaltyStretchCorner.y);
    defaultPositions.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, -penaltyStretchCorner.y);

    return defaultPositions;
}

std::vector<Vector2> OffensiveCoach::getNewOffensivePositions() {

    std::vector<Vector2> defaultLocations = getDefaultLocations();
    if (offensivePositions.size() != defaultLocations.size()) {
        offensivePositions = {};
        for (auto& defaultLocation : defaultLocations) {
            OffensivePosition offensivePosition;
            offensivePosition.position = defaultLocation;
            offensivePosition.score = CoachHeuristics::calculatePositionScore(defaultLocation);
            offensivePositions.emplace_back(offensivePosition);
        }
    } else {
        for (int i = 0; i < offensivePositions.size(); i++) {
            OffensivePosition offensivePosition = offensivePositions[i];
            Vector2 defaultPosition = defaultLocations[i];

            offensivePositions[i] = calculateNewRobotPosition(offensivePosition, defaultPosition);
        }
    }

    drawOffensivePoints();
    return getOffensivePositionVectors();
}

std::vector<Vector2> OffensiveCoach::getOffensivePositionVectors() {
    std::vector<Vector2> positionVectors;
    for (auto& offensivePosition : offensivePositions) {
        positionVectors.emplace_back(offensivePosition.position);
    }
    return positionVectors;
}

}
}
}
