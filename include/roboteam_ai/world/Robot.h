//
// Created by thijs on 1-4-19.
//

#ifndef ROBOTEAM_AI_ROBOT_H
#define ROBOTEAM_AI_ROBOT_H

#include <gtest/gtest_prod.h>
#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "roboteam_proto/WorldRobot.pb.h"

#include "Team.h"
#include "utilities/Constants.h"

namespace rtt::ai {

// control forward declarations
namespace control {
class ShotController;
class NumTreePosControl;
class BallHandlePosControl;
class BasicPosControl;
}  // namespace control

namespace world {

class Ball;
class Robot {
    FRIEND_TEST(ShotControllerTest, getshotdata_test);

   public:
    using BallPtr = std::shared_ptr<Ball>;
    using RobotPtr = std::shared_ptr<Robot>;
    // pid
   private:
    Vector2 pidPreviousVel = Vector2();

   public:
    void setPidPreviousVel(const Vector2 &vel);
    const Vector2 &getPidPreviousVel() const;

    // ball possession
   private:
    double distanceToBall;
    bool iHaveBall;
    unsigned long lastUpdatedWorldNumber = 0;

   public:
    double calculateDistanceToBall(const Vector2 &ballPos);
    bool hasBall(double maxDist = Constants::MAX_BALL_BOUNCE_RANGE()) const;
    double getDistanceToBall();

    // geneva
   private:
    int genevaState;
    int previousGenevaState = 0;
    double timeGenevaChanged = 0;
    double timeToChangeOneGenevaState = 0.2;

   public:
    void setTimeToChangeOneGenevaState(double timeToChangeOneGenevaState);

   private:
    bool workingGeneva;
    bool batteryLow = false;
    double lastReceivedFeedbackMoment = INT_MIN;

   public:
    bool isBatteryLow() const;
    void setBatteryLow(bool batteryLow);
    bool hasRecentFeedback();
    void UpdateFeedbackReceivedTime();

   public:
    void setWorkingGeneva(bool workingGeneva);
    void setHasWorkingBallSensor(bool hasWorkingBallSensor);

   public:
    int getGenevaState() const;
    bool isGenevaReady() const;
    void setGenevaState(int state);
    void setGenevaStateFromFeedback(int state);
    bool genevaStateIsValid(int state);
    bool genevaStateIsDifferent(int state);
    bool hasWorkingGeneva() const;
    bool hasWorkingBallSensor() const;

    // dribbler
   private:
    unsigned char dribblerState = 0;
    unsigned char previousDribblerState = 0;
    double timeDribblerChanged = 0;
    constexpr static double timeToChangeOneDribblerLevel = 0.18;
    bool workingDribbler;
    bool workingBallSensor;

   public:
    unsigned char getDribblerState() const;
    bool isDribblerReady() const;
    void setDribblerState(unsigned char dribbler = 0);
    bool hasWorkingDribbler() const;
    Vector2 getKicker() const;

    // control managers
   private:
    std::shared_ptr<control::ShotController> shotController;
    std::shared_ptr<control::NumTreePosControl> numTreePosControl;
    std::shared_ptr<control::BasicPosControl> basicPosControl;
    std::shared_ptr<control::BallHandlePosControl> ballHandlePosControl;

   public:
    const std::shared_ptr<control::ShotController> &getShotController() const;
    const std::shared_ptr<control::NumTreePosControl> &getNumtreePosControl() const;
    const std::shared_ptr<control::BasicPosControl> &getBasicPosControl() const;
    const std::shared_ptr<control::BallHandlePosControl> &getBallHandlePosControl() const;

    void resetShotController();
    void resetNumTreePosControl();
    void resetBasicPosControl();
    void resetBallHandlePosControl();

    // general
   public:
    Robot();
    explicit Robot(const proto::WorldRobot &copy, Team team = invalid, unsigned char genevaState = 3, unsigned char dribblerState = 0, unsigned long worldNumber = 0);
    void updateRobot(const proto::WorldRobot &robotMsg, const BallPtr &ball, unsigned long worldNumber);
    const unsigned long getLastUpdatedWorldNumber() const;

    int id = -1;
    Angle angle = Angle();
    Vector2 pos = Vector2();
    Vector2 vel = Vector2();
    double angularVelocity = 0.0;
    Team team;
};

}  // namespace world
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ROBOT_H