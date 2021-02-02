//
// Created by Max on 01-01-2021
//

#ifndef RTT_DMGHOSTING_H
#define RTT_DMGHOSTING_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class DMGhosting : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    DMGhosting(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_DMGHOSTING_H
