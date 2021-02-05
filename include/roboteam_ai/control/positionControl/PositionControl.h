//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include "CollisionDetector.h"
#include "control/positionControl/BBTrajectories/WorldObjects.h"
#include "control/RobotCommand.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/PidTracking.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * The main position control class. Use this for your robot position control
 * requirements.
 */
    class PositionControl {
    private:
        /// the distance to the target position at which the robot will stop if it
        /// detects a collision (e.g. target is inside a robot)
        static constexpr double FINAL_AVOIDANCE_DISTANCE = 4 * Constants::ROBOT_RADIUS();

        CollisionDetector collisionDetector;
        rtt::BB::WorldObjects worldObjects;
        NumTreesPlanning pathPlanningAlgorithm = NumTreesPlanning(collisionDetector);
        PidTracking pathTrackingAlgorithm;

        std::unordered_map<int, BB::BBTrajectory2D> computedPaths;

    public:
        /**
         * Generates a path according to the selected planning algorithm,
         * and tracks it using the selected tracking algorithm. In the case a collision
         * is detected (using the collision detector), the path is recalculated.
         * @param field the field object, used onwards by the collision detector
         * @param robotId the ID of the robot for which the path is calculated
         * @param currentPosition the current position of the aforementioned robot
         * @param currentVelocity its velocity
         * @param targetPosition the desired position that the robot has to reach
         * @param pidType The desired PID type (intercept, regular, keeper etc.)
         * @return a RobotCommand, which can be fed directly in the output
         */
        RobotCommand computeAndTrackPath(const rtt_world::Field &field, int robotId, const Vector2 &currentPosition,
                                         const Vector2 &currentVelocity, const Vector2 &targetPosition,
                                         stp::PIDType pidType);

        /**
         * Updates the robot view vector
         * @param robotPositions the position vector of the robots
         */
        void setRobotPositions(std::vector<Vector2> &robotPositions);


        /**
         * The computed path should be recalculated if: <br>
         * - it is empty (no path yet) <br>
         * - the target position changed with at least MAX_TARGET_DEVIATION <br>
         * - the robot will collide with another one by the next path point (ignored if the robot is not moving)
         * @param targetPos final target position
         * @param robotId the ID of the current robot
         * @return true if one of the above conditions are true, false otherwise
         */
        bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                                   const Vector2 &currentVelocity, int robotId);

        BB::BBTrajectory2D
        computePath(const rtt::world::Field &field, Vector2 currentPosition, Vector2 currentVelocity,
                    Vector2 targetPosition, int robotId);

        std::optional<BB::BBTrajectory2D>
        findNewPath(Vector2 &currentPosition, Vector2 &currentVelocity,
                    std::optional<BB::CollisionData> &firstCollision,
                    Vector2 &targetPosition, const rtt::world::Field &field, int robotId, double timeStep);

        std::vector<Vector2>
        createIntermediatePoints(std::optional<BB::CollisionData> &firstCollision, Vector2 &targetPosition,
                                 const rtt::world::Field &field, int robotId);

        std::priority_queue<std::pair<double, Vector2>, std::vector<std::pair<double, Vector2>>, std::greater<>>
        sortIntermediatePoints(std::vector<Vector2> &intermediatePoints,
                               std::optional<BB::CollisionData> &firstCollision);

        //If no collision on path to intermediatePoint, create new points along this path in timeStep increments.
        //Then loop through these new points, generate paths from these to the original target and check for collisions.
        //Return the first path without collisions, or pop() this point and start checking the next one.
        std::optional<BB::BBTrajectory2D>
        calculatePathFromNewStart(std::optional<BB::CollisionData> intermediatePathCollision,
                                  BB::BBTrajectory2D pathToIntermediatePoint, Vector2 &targetPosition, int robotId,
                                  double timeStep);
    };

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
