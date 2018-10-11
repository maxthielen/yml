//
// Created by mrlukasbos on 10-10-18.
//

#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"

namespace rtt {
namespace ai {

/**
 * \class Condition
 * \brief Base class for conditions.
 */
 class Condition : public bt::Leaf {
 public:
  Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
  virtual Status Update();
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
