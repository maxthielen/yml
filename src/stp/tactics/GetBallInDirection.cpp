//
// Created by jordi on 06-04-20.
/// Updated by Max on 03-02-21.
//

#include <roboteam_utils/LineSegment.h>
#include "stp/tactics/GetBallInDirection.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {
GetBallInDirection::GetBallInDirection() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate(), skill::GoToPos()}; }

std::optional<StpInfo> GetBallInDirection::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getPositionToShootAt()) return std::nullopt;

    if (skills.current_num() == 2 && skillStpInfo.getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL ){
        skillStpInfo.setExtraMargin(control_constants::ROBOT_CLOSE_TO_POINT);
    }

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();
    Vector2 targetPosition = info.getPositionToShootAt().value();

    /// The robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition;

    /// If ball is to far away, dont get stuck on other GoToPos(), but restart and get behind the ball
    if (skillStpInfo.getBall().value()->getPos().dist2(skillStpInfo.getRobot().value()->getPos()) > control_constants::DISTANCE_TO_ROBOT_FAR){
        skills.skip_to(0);
    }

    if (skills.current_num() == 0) {

        newRobotPosition = ballPosition + (ballPosition - targetPosition).stretchToLength(
                control_constants::DISTANCE_TO_ROBOT_NEAR);

        /// Check if the path crosses through the ball
        auto pathToBehindBall = LineSegment(robotPosition, newRobotPosition);
        auto distFromPathToBall = pathToBehindBall.distanceToLine(ballPosition);
        auto drivingDirection = robotPosition - newRobotPosition;
        if(distFromPathToBall < control_constants::ROBOT_RADIUS * 1.2) {
            /// Decide which side of the ball to approach on
            // TODO make more efficient, test if calc'ing angle and determining it that way is faster - Choosing best position is also determined by current velocity into which direction
            auto posBallLeft = ballPosition + Vector2(drivingDirection.y, -drivingDirection.x).stretchToLength(control_constants::DISTANCE_TO_ROBOT_NEAR);
            auto posBallRight = ballPosition + Vector2(-drivingDirection.y, drivingDirection.x).stretchToLength(control_constants::DISTANCE_TO_ROBOT_NEAR);
            newRobotPosition = (posBallLeft.dist(robotPosition) < posBallRight.dist(robotPosition)) ? posBallLeft : posBallRight;
        }

    } else {
        // Second GoToPos: Go towards ball
        newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance);

        // Rotate towards ball
        skillStpInfo.setAngle((ballPosition - robotPosition).angle());
    }

    // Turn on dribbler when close to ball
    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    skillStpInfo.setPositionToMoveTo(newRobotPosition);

    return skillStpInfo;
}

bool GetBallInDirection::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt(); }

bool GetBallInDirection::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool GetBallInDirection::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBallInDirection::getName() { return "Get Ball In Direction"; }

}  // namespace rtt::ai::stp::tactic
