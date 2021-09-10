//
// Created by alexander on 30-08-21.
//


/// THIS IS MEANT AS A TEMPLATE FOR MAKING PLAYS. DO NOT ADD THIS FILE TO CMAKE.

/// --------- USAGE:
/// 1. Edit the code below the green comments (or ///)
//  2. Dont touch the code with grey comments (or //)
/// 3. Follow the instructions of each comment
/// 4. Remove the unused optional functions

#include "stp/plays/dance/Dance.h"

#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/active/Dancer.h"

namespace rtt::ai::stp::play {
const char* Dance::getName() { return "Dance"; }

Dance::Dance() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear(); // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear(); // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Dancer>("dancer_0"),
        std::make_unique<role::Dancer>("dancer_1"),
        std::make_unique<role::Dancer>("dancer_2"),
        std::make_unique<role::Dancer>("dancer_3"),
        std::make_unique<role::Dancer>("dancer_4"),
        std::make_unique<role::Dancer>("dancer_5"),
        std::make_unique<role::Dancer>("dancer_6"),
        std::make_unique<role::Dancer>("dancer_7"),
        std::make_unique<role::Dancer>("dancer_8"),
        std::make_unique<role::Dancer>("dancer_9"),
        std::make_unique<role::Dancer>("dancer_10")};
    initRoles(); // DONT TOUCH.
}

Dealer::FlagMap Dance::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap; // DONT TOUCH.

    /// Flags that have a factor and a weight linked to it, can be given to a role

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"dancer_0", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_1", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_2", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_3", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_4", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_5", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_6", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_7", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_8", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_9", {DealerFlagPriority::HIGH_PRIORITY, {}}});
    flagMap.insert({"dancer_10", {DealerFlagPriority::HIGH_PRIORITY, {}}});

    return flagMap; // DONT TOUCH.
}

uint8_t Dance::score(PlayEvaluator& playEvaluator) noexcept {
    /// calculateInfoForScoredRoles(playEvaluator.getWorld()); /// DISABLE IF NOT USED
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::BallCloseToUs),1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value(); // DONT TOUCH.
}

void Dance::calculateInfoForRoles() noexcept {
    /// Function where are roles get their information, make sure not to compute roles twice.
    calculateInfoForScoredRoles(world); // DONT TOUCH.

    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["dancer_0"].setPositionToMoveTo(Vector2(0,width/6));
    stpInfos["dancer_1"].setPositionToMoveTo(Vector2(0,width/3));
    stpInfos["dancer_2"].setPositionToMoveTo(Vector2(0,-width/3));
    stpInfos["dancer_3"].setPositionToMoveTo(Vector2(length/3,width/3));
    stpInfos["dancer_4"].setPositionToMoveTo(Vector2(length/3,0));
    stpInfos["dancer_5"].setPositionToMoveTo(Vector2(length/3,-width/3));
    stpInfos["dancer_6"].setPositionToMoveTo(Vector2(-length/3,width/3));
    stpInfos["dancer_7"].setPositionToMoveTo(Vector2(-length/3,0));
    stpInfos["dancer_8"].setPositionToMoveTo(Vector2(-length/3,-width/3));
    stpInfos["dancer_9"].setPositionToMoveTo(Vector2(-length/2,0));
    stpInfos["dancer_10"].setPositionToMoveTo(Vector2(0,-width/6));

    stpInfos["dancer_0"].setAngle(0);
    stpInfos["dancer_1"].setAngle(0);
    stpInfos["dancer_2"].setAngle(0);
    stpInfos["dancer_3"].setAngle(0);
    stpInfos["dancer_4"].setAngle(0);
    stpInfos["dancer_5"].setAngle(0);
    stpInfos["dancer_6"].setAngle(0);
    stpInfos["dancer_7"].setAngle(0);
    stpInfos["dancer_8"].setAngle(0);
    stpInfos["dancer_9"].setAngle(0);
    stpInfos["dancer_10"].setAngle(0);

}

/// OPTIONAL -> place to compute extra evaluations to end a play
//bool PlayTemplate::shouldEndPlay() noexcept {
//    return false;
//}

/// OPTIONAL -> place to save information for next play
//void PlayTemplate::storePlayInfo(PlayInfos& info) noexcept {
//    StoreInfo role_0;
//    role_0.robotID = stpInfos["role_0"].getRobot()->get()->getId();
//    role_0.moveToPosition = stpInfos["role_0"].getPositionToMoveTo();
//    info.insert({gen::KeyInfo::isShooter, role_0});
//}
}  // namespace rtt::ai::stp::play

