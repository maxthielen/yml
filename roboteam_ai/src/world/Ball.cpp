//
// Created by thijs on 1-4-19.
//

#include "Ball.h"
#include "World.h"
#include "WorldData.h"

#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace world {

Ball::Ball()
    : pos(Vector2()), vel(Vector2()), visible(false) { }

Ball::Ball(const roboteam_msgs::WorldBall &copy)
        : pos(copy.pos), vel(copy.vel),
        visible(copy.visible) {
    exists = exists || copy.existence || Vector2(copy.pos).isNotNaN();
    if (!exists) std::cout << "Ball message has existence = 0!!" << std::endl;
}

const roboteam_msgs::WorldBall Ball::toMessage() const {
    roboteam_msgs::WorldBall ballMsg;
    ballMsg.existence = static_cast<unsigned int>(exists);
    ballMsg.visible = static_cast<unsigned char>(visible);
    ballMsg.pos = pos;
    ballMsg.vel = vel;
    return ballMsg;
}

void Ball::updateBall(const Ball &oldBall, const WorldData &worldData) {
    updateBallModel(oldBall, worldData);
    updateBallPosition(oldBall, worldData);
}

void Ball::updateBallModel(const Ball &oldBall, const WorldData &worldData) {
//TODO: LITERALLY UNREADABLE ???? fix this :)

    // check for collisions
    Vector2 oldVelocity = oldBall.vel;

    //TODO: this model need to be tuned for real life / the noise levels need to be tested esp


    //noise rectangle; rectangle within the velocity state diagram where the velocity is still considered 'good'
    //We consider the linear direction (direction the ball is rolling it) and the direction orthogonal to the linear direction

    //TODO: fix the names of these constants, maybe a comment with their use
    //TODO: did you use a source online for the mathematics? maybe reference that website
    const double linearThreshold = 0.1;
    const double baseLinear = 0.1;
    const double linearSlope = 0.15;
    const double orthogonalThreshold = 0.1;
    const double baseOrthogonal = 0.1;
    const double orthogonalSlope = 0.01;
    const double maxDeceleration = 5.0; // m/s^2

    double linearLength = 0;
    double linearDeviation;
    double orthogonalDeviation;

    if (oldVelocity.length() != 0.0 && vel.length() != 0.0) {
        Vector2 projection = vel.project2(oldVelocity);
        linearLength = projection.length();
    }
    double hypotenuseLength = vel.length();
    double orthogonalLength = 0;
    if (hypotenuseLength >= linearLength) {
        orthogonalLength = sqrt(hypotenuseLength*hypotenuseLength - linearLength*linearLength);
    }

    if (linearLength < linearThreshold) {
        linearDeviation = baseLinear;
    }
    else {
        linearDeviation = baseLinear + linearSlope*(linearLength - linearThreshold);
    }
    if (orthogonalLength < orthogonalThreshold) {
        orthogonalDeviation = baseOrthogonal;
    }
    else {
        orthogonalDeviation = baseOrthogonal + orthogonalSlope*(orthogonalLength - orthogonalThreshold);
    }

    // defining the rectangle around the velocity for which the deviation is acceptable (is always faced towards the origin)
    Vector2 centerLow = oldVelocity - oldVelocity.stretchToLength(linearDeviation + maxDeceleration/60);
    Vector2 centerHigh = oldVelocity + oldVelocity.stretchToLength(linearDeviation);
    Vector2 orthogonalVector = Vector2(orthogonalDeviation, 0).rotate(oldVelocity.angle() + M_PI_2);
    Vector2 point1 = centerLow + orthogonalVector;
    Vector2 point2 = centerLow - orthogonalVector;
    Vector2 point3 = centerHigh - orthogonalVector;
    Vector2 point4 = centerHigh + orthogonalVector;

    // check the noise rectangle
    if (control::ControlUtils::pointInRectangle(vel, point1, point2, point3, point4)) {
        collidesNow = false;
        ballStraightTicks ++;
        kickedNow = false;
    }
    else {
        ballStraightTicks = 0;
        // kicked or collided
        if (vel.length() > (oldVelocity.length() + linearDeviation)) {
            kickedNow = true;
        }
        else {
            collidesNow = true;
        }
    }

    updateDribbling(oldBall, worldData);
}

void Ball::updateDribbling(const Ball &oldBall, const WorldData &worldData) {
    //TODO: tune constants

    double maxDribbleRange = 0.05;
    double maxSpeedDiff = 0.51;

    std::vector<Robot> allRobots;
    allRobots.insert(allRobots.end(), worldData.us.begin(), worldData.us.end());
    allRobots.insert(allRobots.end(), worldData.them.begin(), worldData.them.end());

    Robot* dribblingRobot = getDribblingRobot(allRobots, maxDribbleRange);

    if (! dribblingRobot) {
        dribbledNow = false;
        return;
    }
    dribbledNow = (vel - dribblingRobot->vel).length() < maxSpeedDiff;
    // the ball is being dribbled if the speed of the ball is not too different from the speed of the dribbling robot
}

Robot* Ball::getDribblingRobot(const std::vector<Robot> &robots, double maxDribbleRange) {
    double closestDribbleRange = maxDribbleRange;

    Robot* dribblingRobot = nullptr;
    for (auto robot : robots) {
        if (robot.hasBall(Constants::MAX_BALL_BOUNCE_RANGE())) {
            if (robot.getDistanceToBall() < closestDribbleRange) {
                closestDribbleRange = robot.getDistanceToBall();
                dribblingRobot = &robot;
            }
        }
    }

    return dribblingRobot;
}

void Ball::updateBallPosition(const Ball &oldBall, const WorldData &worldData) {
    if (visible) {
        return;
    }
    RobotPtr robotWithBall = world->whichRobotHasBall();
    if (robotWithBall) {
        std::pair<int, Robot::Team> newRobotIdTeam = {robotWithBall->id, robotWithBall->team};
        Robot newRobotWithBall;
        bool newRobotStillExistsInWorld = false;
        for (auto &robot : worldData.us) {
            if (robot.id == newRobotIdTeam.first && robot.team == newRobotIdTeam.second) {
                newRobotWithBall = robot;
                newRobotStillExistsInWorld = true;
            }
        }
        if (newRobotStillExistsInWorld) {
            double distanceInFrontOfRobot = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
            pos = newRobotWithBall.pos + newRobotWithBall.angle.toVector2(distanceInFrontOfRobot);
        }
    }
}

bool Ball::exists = false;

}
}
}