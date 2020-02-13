#pragma once

#ifndef ROBOTEAM_AI_WORLD_H
#define ROBOTEAM_AI_WORLD_H

#include <roboteam_utils/Vector2.h>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/WorldRobot.pb.h"

#include "Field.h"
#include "Team.h"
#include "WhichRobots.h"
#include "utilities/Constants.h"

namespace rtt::ai::world {

class Robot;
class Ball;
class WorldData;
class History;
class FutureWorld;
class World {
   public:
    using RobotPtr = std::shared_ptr<Robot>;
    using BallPtr = std::shared_ptr<Ball>;
    using WorldDataPtr = std::shared_ptr<WorldData>;

   private:
    WorldDataPtr worldDataPtr;
    std::mutex worldMutex;
    History *history;
    FutureWorld *futureWorld;
    unsigned long worldNumber = 0;
    const RobotPtr getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotPtr> &robots);
    void updateRobotsFromData(Team team, const std::vector<proto::WorldRobot> &robotsFromMsg, std::vector<RobotPtr> &robots, const BallPtr &ball, unsigned long worldNumber) const;

   public:
    explicit World();
    ~World();
    void updateWorld(const Field &field, const proto::World &world);
    bool weHaveRobots();
    double getTimeDifference();
    double getTime();

    // get world
    const WorldData getWorld();
    const WorldData getPreviousWorld();

    // get ball
    BallPtr getBall();

    // get robots
    const RobotPtr getRobotForId(int id, bool ourTeam = true);
    const std::vector<RobotPtr> getRobotsForIds(std::vector<int> ids, bool ourTeam = true);
    const std::vector<RobotPtr> getAllRobots();
    const std::vector<RobotPtr> getUs();
    const std::vector<RobotPtr> getThem();

    // get robot closest to point
    const RobotPtr getRobotClosestToPoint(const Vector2 &point, std::vector<int> robotIds, bool ourTeam);
    const RobotPtr getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots = ALL_ROBOTS);
    const RobotPtr getRobotClosestToBall(WhichRobots whichRobots = ALL_ROBOTS);

    // has ball
    bool robotHasBall(int id, bool ourTeam, double maxDist = Constants::MAX_BALL_RANGE());
    bool ourRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
    bool theirRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
    const RobotPtr whichRobotHasBall(WhichRobots whichRobots = ALL_ROBOTS);

    // future worlds using linear extrapolation
    const WorldData getFutureWorld(double time);
    const RobotPtr getFutureRobot(int id, bool ourTeam, double time);
    const RobotPtr getFutureRobot(const RobotPtr &robot, double time);
    const BallPtr getFutureBall(double time);
};

extern World worldObj;
extern World *world;

}  // namespace rtt::ai::world

#endif  // ROBOTEAM_AI_WORLD_H