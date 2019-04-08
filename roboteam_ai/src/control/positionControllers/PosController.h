//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include "PosVelAngle.h"
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/pid.h>
#include <roboteam_ai/src/world/WorldData.h>

namespace rtt {
namespace ai {
namespace control {
class PosController {
    protected:
        using Robot = world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = world::Ball;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldData = world::WorldData;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        void updatePid(pidVals pid);
    public:
        explicit PosController() = default;
        explicit PosController(bool avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea);
        virtual PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) = 0;
        bool getCanMoveOutOfField() const;
        void setCanMoveOutOfField(bool canMoveOutOfField);
        bool getCanMoveInDefenseArea() const;
        void setCanMoveInDefenseArea(bool canMoveInDefenseArea);
        bool getAvoidBall() const;
        void setAvoidBall(bool avoidBall);

        std::tuple<double, double, double> lastPid;

    protected:

        // settings
        bool avoidBall = false;
        bool canMoveOutOfField = false;
        bool canMoveInDefenseArea = false;

        PID xpid = PID(1.65, 0, 0.0);
        PID ypid = PID(1.65, 0, 0.0);

        bool getPIDFromInterface = true;
        PosVelAngle controlWithPID(const RobotPtr &robot, PosVelAngle target);
        virtual void checkInterfacePID() = 0;

        virtual Vector2 calculatePIDs(const RobotPtr &robot, PosVelAngle &target);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
