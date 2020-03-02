//
// Created by ratoone on 18-11-19.
//

#include <iostream>
#include <chrono>
#include <zconf.h>

#include "control/positionControl/PositionControl.h"
#include "control/positionControl/PositionControlUtils.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"
#include "control/positionControl/pathTracking/PurePursuit.h"
#include "interface/api/Input.h"

namespace rtt::ai::control {
RobotCommand PositionControl::computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition) {
    collisionDetector.setField(field);
//    if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
//        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
//    }

    Vector2 pursuitPoint = computedPaths[robotId].front();
    if(2 <= computedPaths[robotId].size()) {

//        auto t1 = std::chrono::high_resolution_clock::now();
        float pursuitDistance = std::max(currentVelocity.length(), 0.1);
        std::cout << "[PositionControl::computeAndTrackPath] " << pursuitDistance << std::endl;
        std::pair<Vector2, int> _pair = PurePursuit::getPursuingPoint(computedPaths[robotId], currentPosition, pursuitDistance, true, robotId);
//        auto t2 = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
//        std::cout << duration << std::endl;

        pursuitPoint = _pair.first;
        for(int i = 0; i < _pair.second; i++){
            computedPaths[robotId].erase(computedPaths[robotId].begin());
        }

        interface::Input::drawData(interface::Visual::PATHFINDING, {currentPosition, pursuitPoint}, Qt::red, robotId,interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
        interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

        auto pidTracker = pathTrackingAlgorithm.pidTracking;
        pidTracker.updatePidValuesFromInterface();

        Vector2 velocity;
        velocity.x = pidTracker.xPid.getOutput(currentPosition.x, pursuitPoint.x);
        velocity.y = pidTracker.yPid.getOutput(currentPosition.y, pursuitPoint.y);
        Position trackingVelocity(velocity, (pursuitPoint - currentPosition).angle());

        RobotCommand command = RobotCommand();
        command.pos = pursuitPoint;
        command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
        command.angle = trackingVelocity.rot;
        return command;
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);


    RobotCommand command = RobotCommand();
    command.pos = computedPaths[robotId].front();
    Position trackingVelocity = pathTrackingAlgorithm.trackPath(currentPosition, currentVelocity, computedPaths[robotId]);
    command.vel = Vector2(trackingVelocity.x, trackingVelocity.y);
    command.angle = trackingVelocity.rot;

    return command;
}

bool PositionControl::shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                                            const Vector2 &currentVelocity, int robotId) {
    return computedPaths[robotId].empty() || PositionControlUtils::isTargetChanged(targetPos, computedPaths[robotId].back()) ||
            (currentVelocity != Vector2() && collisionDetector.getRobotCollisionBetweenPoints(currentPosition, computedPaths[robotId].front()));
}

void PositionControl::setRobotPositions(std::vector<Vector2> &robotPositions) {
    collisionDetector.setRobotPositions(robotPositions); }
}  // namespace rtt::ai::control