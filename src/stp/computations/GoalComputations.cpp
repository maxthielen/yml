//
// Created by maxl on 11-02-21.
//

#include "stp/computations/GoalComputations.h"
#include "stp/StpInfo.h"

#include <roboteam_utils/Grid.h>

#include <include/roboteam_ai/world/Field.h>
#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::stp::computations {
      Vector2 GoalComputations::calculateGoalTarget(rtt_world::World *world, const rtt_world::Field &field) {
        // Position of the ball from which the goal target is determined
        auto sourcePoint = world->getWorld().value().getBall().value()->getPos();

        // Get the longest line section on the visible part of the goal
        std::vector<LineSegment> openSegments = FieldComputations::getVisiblePartsOfGoal(field, false, sourcePoint, world->getWorld().value().getUs());

        // If there is no empty location to shoot at, just shoot at the center of the goal
        /// TODO-Max communicate this to the play
        if (openSegments.empty()) return field.getTheirGoalCenter();

        // The longest open segment of the goal will be the best to shoot at
        LineSegment bestSegment = getLongestSegment(openSegments);

        // Make two aim points which are in the corners, since these points are harder for the keeper to intercept
        LineSegment aimPoints = getAimPoints(field, sourcePoint);
        auto leftPoint = aimPoints.start;
        auto rightPoint = aimPoints.end;

        // Check if the left and right points are in the best segment
        double maxY = std::max(bestSegment.start.y, bestSegment.end.y);
        double minY = std::min(bestSegment.start.y, bestSegment.end.y);
        bool leftPointInSegment = leftPoint.y < maxY && leftPoint.y > minY;
        bool rightPointInSegment = rightPoint.y < maxY && rightPoint.y > minY;

        // If we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
        if (leftPointInSegment && rightPointInSegment) {
            // Open goal (mostly), so just shoot in the middle of the largest open segment
            return (bestSegment.start + bestSegment.end) * 0.5;
        } else if (leftPointInSegment) {
            return leftPoint;
        } else if (rightPointInSegment) {
            return rightPoint;
        } else {
            return (bestSegment.start + bestSegment.end) * 0.5;
        }
    }

    LineSegment GoalComputations::getAimPoints(const world::Field &field, const Vector2 &sourcePoint) {
        LineSegment goalSides = FieldComputations::getGoalSides(field, false);

        // Aim points are located some distance away from the edges of the goal to take into account inaccuracies in the shot
        const double angleMargin = sin(2.0 / 180.0 * M_PI);
        const double constantMargin = 0.05 * field.getGoalWidth();
        Vector2 leftPoint(goalSides.start.x, goalSides.start.y + constantMargin + angleMargin * goalSides.start.dist(sourcePoint));
        Vector2 rightPoint(goalSides.end.x, goalSides.end.y - angleMargin * goalSides.end.dist(sourcePoint) - constantMargin);

        return LineSegment(leftPoint, rightPoint);
    }

    LineSegment &GoalComputations::getLongestSegment(const std::vector<LineSegment> &openSegments) {
        unsigned bestIndex = 0;
        for (unsigned i = 1; i < openSegments.size(); i++) {
            auto segment = openSegments[i];
            auto bestSegment = openSegments[bestIndex];
            if (fabs(segment.start.y - segment.end.y) > fabs(bestSegment.start.y - bestSegment.end.y)) {
                bestIndex = i;
            }
        }
        return const_cast<LineSegment &>(openSegments[bestIndex]);
    }
} //namespace computations