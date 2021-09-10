//
// Created by tijmen on 30-08-21.
//

//
// Created by maxl on 29-03-21.
//

/// THIS IS MEANT AS A TEMPLATE FOR MAKING PLAYS. DO NOT ADD THIS FILE TO CMAKE.

/// --------- USAGE:
/// 1. Edit the code below the green comments (or ///)
//  2. Dont touch the code with grey comments (or //)
/// 3. Follow the instructions of each comment
/// 4. Remove the unused optional functions

#include "stp/plays/celebrations/danceOff.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/Waller.h"
#include "stp/computations/PositionComputations.h"

namespace rtt::ai::stp::play {
const char* DanceOff::getName() { return "Dance Off"; }

DanceOff::DanceOff() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear(); // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear(); // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Formation>("role_10"),
        std::make_unique<role::Formation>("role_0"),
        std::make_unique<role::Formation>("role_1"),
        std::make_unique<role::Formation>("role_2"),
        std::make_unique<role::Formation>("role_3"),
        std::make_unique<role::Formation>("role_4"),
        std::make_unique<role::Formation>("role_5"),
        std::make_unique<role::Formation>("role_6"),
        std::make_unique<role::Formation>("role_7"),
        std::make_unique<role::Formation>("role_8"),
        std::make_unique<role::Formation>("role_9")};
    initRoles(); // DONT TOUCH.
}

Dealer::FlagMap DanceOff::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap; // DONT TOUCH.

    /// Flags that have a factor and a weight linked to it, can be given to a role
    Dealer::DealerFlag flag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"role_10", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"role_0", {DealerFlagPriority::HIGH_PRIORITY, {flag}}});
    flagMap.insert({"role_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"role_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"role_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"role_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"role_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"role_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"role_7", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_8", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_9", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap; // DONT TOUCH.
}

uint8_t DanceOff::score(PlayEvaluator& playEvaluator) noexcept {
    calculateInfoForScoredRoles(playEvaluator.getWorld()); /// DISABLE IF NOT USED
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::BallCloseToUs),1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
}

/// OPTIONAL -> place to calculateInfoForRoles. Make sure not to compute twice.
void DanceOff::calculateInfoForScoredRoles(world::World* world) noexcept {
    //stpInfos["role_0"].setPositionToMoveTo(pos::getPosition(stpInfos["role_0"].getPositionToMoveTo(),gen::gridRightMid, gen::SafePosition, world->getField().value(), world));
}



void DanceOff::calculateInfoForRoles() noexcept {
    /// Function where are roles get their information, make sure not to compute roles twice.
    calculateInfoForScoredRoles(world); // DONT TOUCH.

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["role_0"].setPositionToMoveTo(Vector2(0, -1));
    stpInfos["role_1"].setPositionToMoveTo(Vector2(0, 0));
    stpInfos["role_2"].setPositionToMoveTo(Vector2(0, 1));
    stpInfos["role_3"].setPositionToMoveTo(Vector2(0, 2));
    stpInfos["role_4"].setPositionToMoveTo(Vector2(0, 3));
    stpInfos["role_5"].setPositionToMoveTo(Vector2(-2, 3));
    stpInfos["role_6"].setPositionToMoveTo(Vector2(-1, 3));
    stpInfos["role_7"].setPositionToMoveTo(Vector2(0, 3));
    stpInfos["role_8"].setPositionToMoveTo(Vector2(1, 3));
    stpInfos["role_9"].setPositionToMoveTo(Vector2(2, 3));
    stpInfos["role_10"].setPositionToMoveTo(Vector2(0, -2));
}


/// OPTIONAL -> place to save information for next play
void DanceOff::storePlayInfo(gen::PlayInfos& info) noexcept {
    /*gen::StoreInfo role_0;
    role_0.robotID = stpInfos["role_0"].getRobot()->get()->getId();
    role_0.moveToPosition = stpInfos["role_0"].getPositionToMoveTo();
    info.insert({gen::KeyInfo::isShooter, role_0});*/
}
}  // namespace rtt::ai::stp::play
