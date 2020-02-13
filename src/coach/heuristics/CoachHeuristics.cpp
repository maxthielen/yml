//
// Created by robzelluf on 3/21/19.
//

#include "coach/heuristics/CoachHeuristics.h"
#include <analysis/GameAnalyzer.h>
#include "control/ControlUtils.h"
#include "world/FieldComputations.h"

namespace rtt::ai::coach {

const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = -0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = -3.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = -3.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = -3.0;
const double CoachHeuristics::DISTANCE_TO_US_WEIGHT = -3.0;
const double CoachHeuristics::ANGLE_TO_GOAL_WEIGHT = -1.0;

const double CoachHeuristics::MAX_INTERCEPT_ANGLE = M_PI / 4.0;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const Field &field, const Vector2 &position) {
    double distanceFromGoal = (field.getTheirGoalCenter() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free.
double CoachHeuristics::calculateShotAtGoalScore(const Field &field, const Vector2 &position, const WorldData &world) {
    WorldData copy = WorldData({}, world.them, world.ball, world.time);
    double viewAtGoal = FieldComputations::getPercentageOfGoalVisibleFromPoint(field, false, position, copy) / 100;
    return 1 - exp(SHOT_AT_GOAL_WEIGHT * viewAtGoal);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2 &position, const WorldData &world) {
    double smallestAngle = MAX_INTERCEPT_ANGLE;
    smallestAngle = getClosestOpponentAngleToPassLine(position, world, smallestAngle);
    return 1 - exp(PASS_LINE_WEIGHT * smallestAngle);
}

double CoachHeuristics::getClosestOpponentAngleToPassLine(const Vector2 &position, const WorldData &world, double smallestAngle) {
    auto ball = world.ball;
    for (const auto &robot : world.them) {
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->getPos(), position)) {
            double angle = abs((position - ball->getPos()).toAngle() - (robot->pos - ball->getPos()).toAngle());
            if (angle < smallestAngle) {
                smallestAngle = angle;
            }
        }
    }
    return smallestAngle;
}

/// Gives a higher score if the position is far away from enemy robots
double CoachHeuristics::calculateDistanceToOpponentsScore(const Vector2 &position) {
    RobotPtr closestRobot = world::world->getRobotClosestToPoint(position, THEIR_ROBOTS);
    if (closestRobot && closestRobot->id != -1) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT * distance);
    } else {
        return 1;
    }
}

double CoachHeuristics::calculateBehindBallScore(const Vector2 &position, const CoachHeuristics::WorldData &world) {
    if (!world.ball) return 0.0;

    double xDistanceBehindBall = world.ball->getPos().x - position.x;
    if (xDistanceBehindBall > 0) {
        return 0.0;
    } else {
        return 1.0;
    }
}

double CoachHeuristics::calculatePassDistanceToBallScore(const Field &field, const Vector2 &position, const CoachHeuristics::WorldData &world) {
    auto ball = world.ball;
    double idealDistance = (field.getTheirGoalCenter() - ball->getPos()).length() * 0.5;
    double distanceFromBall = (position - ball->getPos()).length();

    if (distanceFromBall < Constants::MAX_PASS_DISTANCE()) {
        return -1;
    }

    return fmax(0.0, -pow(distanceFromBall / (0.5 * idealDistance), 2.0) + 2.0 * (distanceFromBall / (0.5 * idealDistance)));
}

double CoachHeuristics::calculatePositionDistanceToBallScore(const Field &field, const Vector2 &position, const CoachHeuristics::WorldData &world) {
    auto ball = world.ball;
    double idealDistance = (field.getTheirGoalCenter() - ball->getPos()).length() * 0.75;
    double distanceFromBall = (position - ball->getPos()).length();
    return fmax(0.0, -pow(distanceFromBall / (0.5 * idealDistance), 2.0) + 2.0 * (distanceFromBall / (0.5 * idealDistance)));
}

double CoachHeuristics::calculateDistanceToClosestTeamMateScore(const Vector2 &position, int thisRobotID) {
    std::vector<int> idVector;
    for (auto &robot : world::world->getUs()) {
        if (robot->id != thisRobotID) {
            idVector.emplace_back(robot->id);
        }
    }

    RobotPtr closestRobot = world::world->getRobotClosestToPoint(position, idVector, true);
    if (closestRobot && closestRobot->id != -1) {
        double distance = (position - closestRobot->pos).length();
        return 1.0 - exp(DISTANCE_TO_US_WEIGHT * distance);
    } else {
        return 1.0;
    }
}

double CoachHeuristics::calculateAngleToGoalScore(const Field &field, const Vector2 &position) {
    auto goalSides = FieldComputations::getGoalSides(field, false);
    Angle angle1 = (goalSides.start - position).toAngle();
    Angle angle2 = (goalSides.end - position).toAngle();

    Angle angleToGoal = abs(angle2 - angle1);

    return 1.0 - exp(ANGLE_TO_GOAL_WEIGHT * angleToGoal);
}

}  // namespace rtt::ai::coach