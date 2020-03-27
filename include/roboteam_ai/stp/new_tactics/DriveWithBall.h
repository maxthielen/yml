//
// Created by timovdk on 3/16/20.
//

#ifndef RTT_DRIVEWITHBALL_H
#define RTT_DRIVEWITHBALL_H

#include <stp/Tactic.h>

namespace rtt::ai::stp::tactic {
class DriveWithBall : public Tactic {
   public:
    DriveWithBall();

   private:
    /**
     * On initialization of this tactic, initialize the state machine with skills
     */
    void onInitialize() noexcept override;

    /**
     * On update of this tactic
     */
    void onUpdate(Status const &status) noexcept override;

    /**
     * On terminate of this tactic, call terminate on all underlying skills
     */
    void onTerminate() noexcept override;

    /**
     * Calculate the SkillInfo from the TacticInfo
     * @param info info is the TacticInfo passed by the role
     * @return SkillInfo based on the TacticInfo
     */
    StpInfo calculateInfoForSkill(StpInfo const &info) noexcept override;

    bool isEndTactic() noexcept override;

    bool isTacticFailing(const StpInfo &info) noexcept override;

    bool shouldTacticReset(const StpInfo &info) noexcept override;
};
}  // namespace rtt::ai::stp::tactic

#endif  // RTT_DRIVEWITHBALL_H