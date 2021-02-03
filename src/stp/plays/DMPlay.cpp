//
// Created by Max on 01-01-2021
//

#include <stp/invariants/BallIsFreeInvariant.h>
#include <stp/invariants/game_states/NormalPlayGameStateInvariant.h>
#include <stp/invariants/BallShotOrCloseToThemInvariant.h>
#include "stp/plays/DMPlay.h"

#include "stp/invariants/BallOnOurSideInvariant.h"
#include "stp/roles/DMGhosting.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/Attacker.h"

namespace rtt::ai::stp::play {

DMPlay::DMPlay() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());

    keepPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::DMGhosting>(role::DMGhosting("notkeeper")),          std::make_unique<role::DMGhosting>(role::DMGhosting("defender_1")),
        std::make_unique<role::DMGhosting>(role::DMGhosting("defender_2")),  std::make_unique<role::DMGhosting>(role::DMGhosting("defender_3")),
        std::make_unique<role::DMGhosting>(role::DMGhosting("defender_4")),   std::make_unique<role::DMGhosting>(role::DMGhosting("defender_5")),
        std::make_unique<role::DMGhosting>(role::DMGhosting("defender_6")),   std::make_unique<role::DMGhosting>(role::DMGhosting("defender_7")),
        std::make_unique<role::DMGhosting>(role::DMGhosting("defender_8")),    std::make_unique<role::DMGhosting>(role::DMGhosting("defender_9")),
        std::make_unique<role::Attacker>(role::Attacker("shooter"))};
}

uint8_t DMPlay::score(world::World *world) noexcept { return 100; }

Dealer::FlagMap DMPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag Shooter(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag GoForIt(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"notkeeper", {keeperFlag}});
    flagMap.insert({"defender_1", {GoForIt}});
    flagMap.insert({"defender_2", {GoForIt}});
    flagMap.insert({"defender_3", {GoForIt}});
    flagMap.insert({"defender_4", {GoForIt}});
    flagMap.insert({"defender_5", {GoForIt}});
    flagMap.insert({"defender_6", {GoForIt}});
    flagMap.insert({"defender_7", {notImportant}});
    flagMap.insert({"defender_8", {notImportant}});
    flagMap.insert({"defender_9", {notImportant}});
    flagMap.insert({"shooter", {Shooter}});

    return flagMap;
}

void DMPlay::calculateInfoForRoles() noexcept {
    calculateInfoForShooter();
    calculateInfoForDefenders();
    //calculateInfoForKeeper();
}

void DMPlay::calculateInfoForDefenders() noexcept {
    auto them = world->getWorld()->getThem();
    auto us = world->getWorld()->getUs();
    for (int i = 0; i < roles.size(); i++) {
        auto roleName = roles[i]->getName();
        if (roleName == "keeper") {continue;}
        if (i < them.size()) {stpInfos[roleName].setEnemyRobot(them[i]);}
        else {stpInfos[roleName].setEnemyRobot(them[0]);}
    }
}

void DMPlay::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DMPlay::calculateInfoForShooter() noexcept {
    stpInfos["shooter"].setShotType(ShotType::MAX);
    stpInfos["shooter"].setPositionToShootAt(field.getTheirGoalCenter());
}

bool DMPlay::shouldRoleSkipEndTactic() { return false; }

const char *DMPlay::getName() { return "DMPlay"; }

}  // namespace rtt::ai::stp::play