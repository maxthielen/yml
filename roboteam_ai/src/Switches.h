//
// Created by baris on 15/11/18.
//
#ifndef ROBOTEAM_AI_SWITCHES_H
#define ROBOTEAM_AI_SWITCHES_H

#include "roboteam_ai/src/treeinterp/BTImport.h"
#include <iostream>
#include "roboteam_ai/src/utilities/RobotDealer.h"
#include "roboteam_ai/src/bt/bt.hpp"

class Switches {
        using robotType = robotDealer::RobotType;

    private:
        static void runErrorHandler(std::map<std::string, std::map<std::string, robotType>> tactics);

    public:

        static std::vector<std::string> tacticJsonFileNames;

        static std::vector<std::string> strategyJsonFileNames;

        static std::vector<std::string> keeperJsonFiles;

        static bt::Node::Ptr nonLeafSwitch(std::string name);

        static bt::Node::Ptr leafSwitch(std::string name, bt::Blackboard::Ptr properties);

        static bt::Node::Ptr tacticSwitch(std::string name, bt::Blackboard::Ptr properties);

};

#endif //ROBOTEAM_AI_SWITCHES_H