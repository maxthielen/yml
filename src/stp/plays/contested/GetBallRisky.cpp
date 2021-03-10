//
// Created by timovdk on 5/15/20.
//

#include "include/roboteam_ai/stp/plays/contested/GetBallRisky.h"

#include "stp/invariants/BallClosestToUsInvariant.h"
#include "stp/invariants/BallIsFreeInvariant.h"
#include "stp/invariants/WeHaveMajorityInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "include/roboteam_ai/stp/roles/active/BallGetter.h"
#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "stp/roles/Keeper.h"
#include "include/roboteam_ai/stp/roles/active/PassReceiver.h"

namespace rtt::ai::stp::play {

GetBallRisky::GetBallRisky() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveMajorityInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallIsFreeInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveMajorityInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallClosestToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::BallGetter>(role::BallGetter("ball_getter")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_0")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_1")),
                                                                                 std::make_unique<role::PassReceiver>(role::PassReceiver("receiver_2")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_0")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Defender>(role::Defender("midfielder_0")),
                                                                                 std::make_unique<role::Defender>(role::Defender("midfielder_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("midfielder_2"))};
}

uint8_t GetBallRisky::score(world::World* world) noexcept { return 120; }

void GetBallRisky::calculateInfoForRoles() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world::them);

    if (enemyRobots.empty()) {
        RTT_ERROR("There are no enemy robots, which are necessary for this play!")
        return;
    }

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyAttacker && enemyRobot->getId() == enemyAttacker.value()->getId(); }));

    auto enemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    // TODO: determine better future receive positions
    stpInfos["receiver_0"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, field.getFieldWidth() / 4));
    stpInfos["receiver_1"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4));
    stpInfos["receiver_2"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, 0));

    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToGoal);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    if (enemyClosestToGoal)
        stpInfos["defender_2"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
    else
        stpInfos["defender_2"].setPositionToDefend(field.getTheirGoalCenter() + Vector2{1, 1});
    stpInfos["defender_2"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["midfielder_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["midfielder_0"].setEnemyRobot(enemyAttacker);
    stpInfos["midfielder_0"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["midfielder_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["midfielder_1"].setEnemyRobot(enemyClosestToGoal);
    stpInfos["midfielder_1"].setBlockDistance(BlockDistance::CLOSE);

    if (enemyClosestToGoal)
        stpInfos["midfielder_2"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
    else
        stpInfos["midfielder_2"].setPositionToDefend(field.getTheirGoalCenter() + Vector2{1, 1});
    stpInfos["midfielder_2"].setEnemyRobot(enemyAttacker);
    stpInfos["midfielder_2"].setBlockDistance(BlockDistance::CLOSE);
}

bool GetBallRisky::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap GetBallRisky::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::UNIQUE);
    Dealer::DealerFlag ballGetter(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"ball_getter", {ballGetter}});
    flagMap.insert({"receiver_0", {notImportant}});
    flagMap.insert({"receiver_1", {notImportant}});
    flagMap.insert({"receiver_2", {notImportant}});
    flagMap.insert({"defender_0", {notImportant}});
    flagMap.insert({"defender_1", {notImportant}});
    flagMap.insert({"defender_2", {notImportant}});
    flagMap.insert({"midfielder_0", {notImportant}});
    flagMap.insert({"midfielder_1", {notImportant}});
    flagMap.insert({"midfielder_2", {notImportant}});
    return flagMap;
}

const char* GetBallRisky::getName() { return "Get Ball Risky"; }

}  // namespace rtt::ai::stp::play
