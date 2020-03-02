//
// Created by baris on 10-5-19.
//

#include "skills/DriveWithInterface.h"

#include <interface/api/Output.h>
#include "world_new/World.hpp"

namespace rtt::ai {
DriveWithInterface::DriveWithInterface(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {
    std::random_device rd;
    mt = std::mt19937(rd());
    randDistribution = std::uniform_real_distribution<double>(-4.0, 4.0);
}

void DriveWithInterface::onInitialize() {
    std::cout << "[DriveWithInterface::onInitialize]" << std::endl;
    path.clear();
    srand(time(NULL));

    staticPath.emplace_back(4, 4);
    staticPath.emplace_back(4, -4);
    staticPath.emplace_back(-4, -4);
    staticPath.emplace_back(-4, 4);

    staticPath.emplace_back(-3.5, 4);
    staticPath.emplace_back(-3.5, -3);
    staticPath.emplace_back(-3, -3);
    staticPath.emplace_back(-3, 4);

    for(int y = 4; -3 < y; y--){
        int dx = std::abs((y % 2));
        staticPath.emplace_back(-1.5 - dx, y);
        staticPath.emplace_back(-1.5 - dx, y-1);
    }

    staticPath.emplace_back(-1, -3);
    for(int y = -3; y < 4; y++){
        staticPath.emplace_back(-0.5, y);
        staticPath.emplace_back(-0.5, y+0.5);
        staticPath.emplace_back(-1, y+0.5);
        staticPath.emplace_back(-1, y+1);
    }

    staticPath.emplace_back(0.5, 4);
    for(int y = 4; 0 <= y; y -= 2){
        staticPath.emplace_back(1.5, y-1);
        staticPath.emplace_back(0.5, y-2);
    }
    staticPath.emplace_back(0.5, -3);

    staticPath.emplace_back(2, -3);
    staticPath.emplace_back(2.1, 0.5);
    staticPath.emplace_back(2, 4);
    staticPath.emplace_back(2.65, 4);
    staticPath.emplace_back(2.75, 0.5);
    staticPath.emplace_back(2.65, -3);
    staticPath.emplace_back(3.4, -3);
    staticPath.emplace_back(3.5, 0.5);
    staticPath.emplace_back(3.4, 4);
}

Skill::Status DriveWithInterface::onUpdate() {
    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    auto controller = world_new::World::instance()->getRobotPositionController();

//    while(controller->computedPaths[robot->id].size() < 10){
//        double x = randDistribution(mt);
//        double y =  randDistribution(mt);
//        controller->computedPaths[robot->id].emplace_back(x, y);
//    }

    while(controller->computedPaths[robot->id].size() <= staticPath.size()){
        controller->computedPaths[robot->id].emplace_back(staticPath.at(iStaticPath));
        iStaticPath++;
        iStaticPath %= staticPath.size();
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

    auto robotCommand = controller->computeAndTrackPath(*field, robot->id, robot->pos, robot->vel, targetPos);

    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);
    publishRobotCommand();
    return Status::Running;
}
}  // namespace rtt::ai