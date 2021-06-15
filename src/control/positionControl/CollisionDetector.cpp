//
// Created by ratoone on 10-12-19.
//

#include "control/positionControl/CollisionDetector.h"

namespace rtt::ai::control {

bool CollisionDetector::isCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint, std::optional<rtt::world::view::RobotView> robot) {
    bool isFieldColliding = field ? !isPointInsideField(nextPoint) || getDefenseAreaCollision(initialPoint, nextPoint) : false;

    //colliding with the outside of the field, the defense area, or collision with a robot
    return isFieldColliding || getRobotCollisionBetweenPoints(initialPoint, nextPoint, robot);
}

std::optional<Vector2> CollisionDetector::getCollisionBetweenPoints(const Vector2& point, const Vector2& nextPoint, std::optional<rtt::world::view::RobotView> robot) {
    auto robotCollision = getRobotCollisionBetweenPoints(point, nextPoint, robot);
    auto defenseCollision = getDefenseAreaCollision(point, nextPoint);

    return (defenseCollision.value_or(nextPoint) - point).length2() < (robotCollision.value_or(nextPoint) - point).length2() ?
            defenseCollision :
            robotCollision;
}

bool CollisionDetector::isPointInsideField(const Vector2& point) { return FieldComputations::pointIsInField(*field, point, Constants::ROBOT_RADIUS()); }

std::optional<Vector2> CollisionDetector::getDefenseAreaCollision(const Vector2 &point, const Vector2 &nextPoint) {
    auto ourDefenseCollision = FieldComputations::lineIntersectionWithDefenceArea(*field, true, point, nextPoint, DEFAULT_ROBOT_COLLISION_RADIUS);
    if (ourDefenseCollision){
        return *ourDefenseCollision;
    }

    auto theirDefenseCollision = FieldComputations::lineIntersectionWithDefenceArea(*field, false, point, nextPoint, DEFAULT_ROBOT_COLLISION_RADIUS);
    if (!theirDefenseCollision) {
        return std::nullopt;
    }
    return *theirDefenseCollision;
}

std::optional<Vector2> CollisionDetector::getRobotCollisionBetweenPoints(const Vector2& initialPoint, const Vector2& nextPoint, std::optional<rtt::world::view::RobotView> robot_) {
    Vector2 ourVel = robot_->get()->getVel();
    for (const auto& robot : robots) {
            Vector2 theirVel = robot->getVel();
            Vector2 posDif = robot->getPos() - robot_->get()->getPos();
            Vector2 velDif = ourVel - theirVel;
            double projectLength = velDif.dot(posDif) / posDif.length();
            // if the initial point is already close to a robot, then either 1. there is a collision, or 2. it is the original robot
            if ((robot->getPos() - initialPoint).length() > Constants::ROBOT_RADIUS() &&
                LineSegment(initialPoint, nextPoint).distanceToLine(robot->getPos()) < DEFAULT_ROBOT_COLLISION_RADIUS
                /*&& (abs(projectLength) > 1.5 && theirVel.length() < ourVel.length())*/) {
                return robot->getPos();
            }
    }

    return std::nullopt;
}

std::vector<rtt::world::view::RobotView> CollisionDetector::getRobots() {
    return robots;
}

void CollisionDetector::setField(const rtt::world::Field& field_) { this->field = &field_; }

void CollisionDetector::setRobots(std::vector<rtt::world::view::RobotView> &robots_) { this->robots = robots_; }

}  // namespace rtt::ai::control