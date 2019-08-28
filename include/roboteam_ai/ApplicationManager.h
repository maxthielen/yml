//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include <include/roboteam_ai/utilities/StrategyManager.h>
#include "include/roboteam_ai/io/IOManager.h"
#include "include/roboteam_ai/treeinterp/BTFactory.h"

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);
    rtt::ai::io::IOManager * IOManager;

    int ticksFree = 0;
    bt::BehaviorTree::Ptr strategy;
    bt::BehaviorTree::Ptr keeperTree;

    void notifyTreeStatus(bt::Node::Status status);
    void runOneLoopCycle();
    bool weHaveRobots = false;

    ai::StrategyManager strategyManager;
    std::string oldKeeperTreeName = "";
    std::string oldStrategyName = "";

public:
    void setup();
    void loop();
    void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
