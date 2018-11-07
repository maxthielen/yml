//
// Created by baris on 06/11/18.
//

#ifndef ROBOTEAM_AI_TACTIC_H
#define ROBOTEAM_AI_TACTIC_H

#include "Node.hpp"
namespace bt {

class Tactic : public Node {

        void Initialize() override;

        Status Update() override;

        void askForRobots();

        void AddChild(Node::Ptr newChild) override;

        void Terminate(Status s) override;

        Node::Ptr child = nullptr;

        std::string name;

        int numberOfRobots = 0;
};
}

#endif //ROBOTEAM_AI_TACTIC_H