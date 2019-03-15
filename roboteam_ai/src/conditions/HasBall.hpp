#ifndef ROBOTEAM_AI_IHAVEBALL_HPP
#define ROBOTEAM_AI_IHAVEBALL_HPP

#include "Condition.h"

namespace rtt {
namespace ai {

class HasBall : public Condition {
private:
    double ballRange;
public:
    explicit HasBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    std::string node_name() override { return "HasBall"; }
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_IHAVEBALL_HPP
