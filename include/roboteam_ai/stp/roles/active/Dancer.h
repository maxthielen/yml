//
// Created by jordi on 17-03-20.
//

#ifndef RTT_DANCER_H
#define RTT_DANCER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Dancer : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Dancer(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_DANCER_H
