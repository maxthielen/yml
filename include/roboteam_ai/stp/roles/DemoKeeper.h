//
// Created by Jaro & Floris on 18-05-21.
//

#ifndef RTT_DEMOKEEPER_H
#define RTT_DEMOKEEPER_H

#include "stp/Role.hpp"
#include "world/Field.h"

namespace rtt::ai::stp::role {

    class DemoKeeper : public Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a state machine of tactics
         * @param name name of the role
         */
        explicit DemoKeeper(std::string name);

        /**
         * Besides the default update from base class Role, it also switches between tactics depending on the ball position and velocity
         * @param info TacticInfo to be passed to update()
         * @return The status that the current tactic returns
         */
        [[nodiscard]] Status update(StpInfo const& info) noexcept override;

       private:
        /**
         * Checks if ball is in our defense area and still
         * @param field Field
         * @param ballPos Ball position
         * @param ballVel Ball velocity
         * @return True if ball is in our defense area and still
         */
        [[nodiscard]] static bool isBallInOurDefenseAreaAndStill(const world::Field& field, const Vector2& ballPos, const Vector2& ballVel) noexcept;

    };
}  // namespace rtt::ai::stp::role

#endif  // RTT_DemoKeeper_H
