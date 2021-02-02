//
// Created by Max on 01-01-2021
//

#ifndef RTT_DMPLAY_H
#define RTT_DMPLAY_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * DMPlay Play is executed when the opponent has or is close to the ball but not necessarily on our side of the field.
 * In this case the opponent most likely will pass to another robot. Our robots will namely block off robots that can
 * be passed to.
 */
class DMPlay : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    DMPlay();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world::World* world) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    bool shouldRoleSkipEndTactic() override;

    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the blockers
     */
    void calculateInfoForShooter() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DMPLAY_H
