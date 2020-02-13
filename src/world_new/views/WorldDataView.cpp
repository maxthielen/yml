//
// Created by john on 1/6/20.
//

#include "world_new/views/WorldDataView.hpp"

#include "world_new/Ball.hpp"
#include "world_new/WorldData.hpp"

namespace rtt::world_new::view {

std::vector<view::RobotView> const &WorldDataView::getUs() const noexcept { return data->getUs(); }

std::vector<view::RobotView> const &WorldDataView::getThem() const noexcept { return data->getThem(); }

std::vector<robot::Robot> const &WorldDataView::getRobots() const noexcept { return data->getRobots(); }

std::optional<view::BallView> WorldDataView::getBall() const noexcept { return data->getBall(); }

std::optional<view::RobotView> WorldDataView::getRobotForId(uint8_t id, bool ourTeam) const noexcept {
    auto const &_data = ourTeam ? getUs() : getThem();  // const&, prevents copy

    // return if the lambda evaluates to true
    auto _rbt = std::find_if(_data.begin(), _data.end(), [id](RobotView rbt) { return rbt->getId() == id; });

    // not found ? None, else Some(*_robot);
    return _rbt == _data.end() ? std::optional<RobotView>() : *_rbt;
}

std::vector<RobotView> WorldDataView::getRobotsForIds(std::set<uint8_t> const &_robots, bool ourTeam) const noexcept {
    auto isInSet = [&](RobotView view) { return static_cast<bool>(_robots.count(view->getId())); };

    std::vector<RobotView> retRobots{};
    auto const &_data = ourTeam ? getUs() : getThem();
    std::copy_if(_data.begin(), _data.end(), retRobots.begin(), isInSet);
    return retRobots;
}

WorldData const &WorldDataView::operator*() const noexcept { return *get(); }

WorldData const *WorldDataView::operator->() const noexcept { return get(); }

WorldData const *WorldDataView::get() const noexcept { return data; }

RobotView WorldDataView::getRobotClosestToPoint(const Vector2 &point, std::set<uint8_t> const &robotIds, bool ourTeam) const noexcept {
    RobotView closestBot{nullptr};
    double maxDist = 9e9;
    for (auto const &id : robotIds) {
        auto robot = getRobotForId(id, ourTeam);
        if (!robot) {
            continue;
        }
        auto _robot = *robot;
        auto dist = _robot->getPos().dist(point);
        if (dist < maxDist) {
            maxDist = dist;
            closestBot = _robot;
        }
    }
    return closestBot;
}

rtt::world_new::view::WorldDataView::operator bool() const noexcept { return get() != nullptr; }

RobotView WorldDataView::getRobotClosestToPoint(const Vector2 &point, Team team) const noexcept {
    RobotView closest{nullptr};
    std::vector<RobotView> robots;
    if (team == us) robots = getUs();
    if (team == them)
        robots = getThem();
    else
        robots = getRobotsNonOwning();

    return getRobotClosestToPoint(point, robots);
}

RobotView WorldDataView::getRobotClosestToBall(Team team) const noexcept { return getRobotClosestToPoint((*getBall())->getPos(), team); }

bool WorldDataView::robotHasBall(uint8_t id, bool ourTeam, double maxDist) const noexcept { return ourTeam ? ourRobotHasBall(id, maxDist) : theirRobotHasBall(id, maxDist); }

bool WorldDataView::ourRobotHasBall(uint8_t id, double maxDist) const noexcept {
    auto robot = getRobotForId(id, true);
    if (!robot) return false;

    return (*robot).hasBall(maxDist);
}

bool WorldDataView::theirRobotHasBall(int id, double maxDist) const noexcept {
    auto robot = getRobotForId(id, false);
    if (!robot) return false;

    return (*robot).hasBall(maxDist);
}

RobotView WorldDataView::whichRobotHasBall(Team team) {
    std::vector<RobotView> robots;
    if (team == us) {
        robots = getUs();
    } else if (team == them) {
        robots = getThem();
    } else {
        robots = getRobotsNonOwning();
    }

    double bestDistance = 9e9;
    RobotView bestRobot = RobotView{nullptr};
    for (auto &robot : robots) {
        if (robot.hasBall()) {
            if (robot->getDistanceToBall() < bestDistance) {
                bestRobot = robot;
            }
        }
    }

    return bestRobot;
}

RobotView WorldDataView::getRobotClosestToPoint(const Vector2 &point, const std::vector<RobotView> &robots) const noexcept {
    if (robots.empty()) return RobotView{nullptr};

    size_t bestIndex = 0;
    double closest = 9e9;
    for (size_t i = 0; i < robots.size(); i++) {
        double distance = (robots[i]->getPos() - point).length();
        if (distance < closest) {
            closest = distance;
            bestIndex = i;
        }
    }

    return robots[bestIndex];
}

const std::vector<RobotView> & WorldDataView::getRobotsNonOwning() const noexcept { return data->getRobotsNonOwning(); }

WorldDataView::WorldDataView(WorldData const *_ptr) noexcept : data{_ptr} {}
}  // namespace rtt::world_new::view