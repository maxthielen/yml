//
// Created by ratoone on 18-11-19.
//

#include "control/positionControl/PositionControl.h"
#include "control/positionControl/PositionControlUtils.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"
#include "control/positionControl/pathTracking/PurePursuit.h"
#include "interface/api/Input.h"

namespace rtt::ai::control {
RobotCommand PositionControl::computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                                  const Vector2 &targetPosition) {
    collisionDetector.setField(field);
    if (shouldRecalculatePath(currentPosition, targetPosition, currentVelocity, robotId)) {
        computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::green, robotId, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::PATHFINDING, computedPaths[robotId], Qt::blue, robotId, interface::Drawing::DOTS);

    Vector2 pursuitPoint = computedPaths[robotId].front();
    if(2 <= computedPaths[robotId].size()) {
        pursuitPoint = PurePursuit::getPursuingPoint(computedPaths[robotId], currentPosition, 0.4);
        interface::Input::drawData(interface::Visual::PATHFINDING, {currentPosition, pursuitPoint}, Qt::red, robotId,interface::Drawing::LINES_CONNECTED);
        computedPaths[robotId].front() = pursuitPoint;
    }

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