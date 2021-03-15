//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYCHECKER_HPP
#define RTT_PLAYCHECKER_HPP

#include <vector>

#include "Play.hpp"

namespace rtt::ai::stp {

/**
 * Class that gets all the viable plays for a tick
 */
class PlayChecker {
   public:
    /**
     * Sets all the plays in the PlayChecker, takes ownership of plays
     * @param plays Moved vector of unique plays
     */
    void setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept;

    /**
     * Gets a vector of currently valid plays
     * @return map(plays, isValid)
     */
    [[nodiscard]] std::vector<Play*> getValidPlays() noexcept;

    /**
     * updates world and playscorer
     * @param world World to update against
     * @param playEvaluator PlayScorer to update against
     */
    void update(world::World* world, PlayEvaluator& playEvaluator) noexcept;

    /**
     * Returns the default play
     * @param playName The name of the play we want to return
     * @return The play with the name of the argument
     */
    [[nodiscard]] Play* getPlayForName(const std::string& playName) const noexcept;

   private:
    /**
     * A vector of all the plays
     */
    std::vector<std::unique_ptr<Play>>* allPlays{};

    /**
     * Current world, do not use before update()
     */
    world::World* world{};

    /**
     * playEvaluator to check invariant to start
     */
     PlayEvaluator playEvaluator;
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAYCHECKER_HPP
