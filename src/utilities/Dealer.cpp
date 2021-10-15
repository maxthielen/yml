/**
 * The dealer will check for the flags that are set in plays, but also for the distance
 * to a position that a robot might need to travel to. The lower the score of a robot, the better.
 */

// TODO Fix issue where roles get redistributed whilst robots are already in position
/// This issue occurs when there are multiple roles classes (defender+midfielder) that have the same priority

#include "utilities/Dealer.h"
#include <iterator>

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Print.h>

#include "utilities/GameStateManager.hpp"
#include "world/FieldComputations.h"

namespace rtt::ai {

    Dealer::Dealer(v::WorldDataView world, rtt_world::Field *field) : world(world), field(field) {}

    Dealer::DealerFlag::DealerFlag(DealerFlagTitle title, DealerFlagPriority priority) : title(title), priority(priority) {}

///Distributes the roles between the friendly robots considering all information given in the flagMap and stpInfoMap
    std::unordered_map<std::string, v::RobotView> Dealer::distribute(std::vector<v::RobotView> unassignedRobots, FlagMap flagMap,
                                                                     const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
        std::unordered_map<std::string, v::RobotView> output;  // declare return variable

        distribute_forcedIDs(unassignedRobots, flagMap, output);  // assign all forced IDs and remove the corresponding robot and role assigned (to reduce future computations)

        std::vector<RoleScores> scores{getScoreMatrix(unassignedRobots, flagMap, stpInfoMap)};  // compute scores of all robots for all roles

        std::vector<std::string> roleNames;                    // stores all role names in the FlagMap
        roleNames.reserve(flagMap.size());                     // reserve space for all roles in flagMap
        for (auto const &[roleName, dealerFlags] : flagMap) {  // loop through all roles in flagMap
            roleNames.push_back(roleName);                     // add role name
        }

        for (const auto &currentPriority : PriorityOrder) {  // Loop through every FlagPriority in order
            DealerDistribute current{};                      // declare distribution struct for current FlagPriority

            // Check if a robot score has the desired priority
            for (std::size_t j = 0; j < scores.size(); j++) {                     // for each remaining role in the score matrix
                if (scores[j].priority == currentPriority) {                      // if the role has the priority we are looking for (currentPriority is one loop up)
                    current.relevant_scores.push_back(scores.at(j).robotScores);  // store score column of current role
                    current.relevant_roleIndices.push_back(j);                    // store the role indices of current.relevant_scores *in relation to the full score matrix* (used for removing assigned roles in distribute_remove)
                    current.relevant_roleNames.push_back(roleNames[j]);           // store the role name of current.relevant_scores


                }
            }

            if (current.relevant_roleNames.empty()) { RTT_DEBUG("[NULL] roles found for currentPriority") }
            else {
                rtt::Hungarian::Solve(current.relevant_scores, current.new_assignments);  // solve score matrix

                if (current.new_assignments.empty()) { RTT_DEBUG("[FAILED] Hungarian::Solve") }
                else {
                    for (std::size_t j = 0; j < current.new_assignments.size(); j++) {
                        if (current.new_assignments[j] >= 0) {                                                               // checks if newAssignment is valid
                            output.insert({current.relevant_roleNames[j], unassignedRobots[current.new_assignments[j]]});  // insert role name and robot into the output
                            //TODO:: tests if relevant_roles and new_assignments match index wise
                        }
                        else { RTT_DEBUG("[INVALID] value found in new_assignment") }
                    }

                    distribute_remove(current, roleNames, scores, unassignedRobots);  // remove newly assigned robots from future computations
                    if (unassignedRobots.size() == 0) return output;                  // return if all robots are assigned (if forced IDs have been assigned)
                }
            }
        }
        return output;
    }

/// Distributes the forced roles first, so that the following functions do not need to compute extra information
    void Dealer::distribute_forcedIDs(std::vector<v::RobotView> &unassignedRobots, FlagMap &flagMap, std::unordered_map<std::string, v::RobotView> &output) {
        for (auto role = flagMap.begin(); role != flagMap.end(); ++role) {  // loop through every role in the flagMap
            int ID = role->second.forcedID;
            if (ID != -1) {                                                                                                                // if ID is a forced ID
                if (!std::any_of(unassignedRobots.begin(), unassignedRobots.end(), [ID](v::RobotView &x) { return x->getId() == ID; })) {  // if the ID is NOT found within allRobots
                    RTT_ERROR("ID " + std::to_string(ID) + " is not a VALID ID. The forced ID will be IGNORED.")
                    continue;
                }
                output.insert({role->first, unassignedRobots[ID]});     // insert role name and robot into the output
                unassignedRobots.erase(unassignedRobots.begin() + ID);  // remove the robot to reduce future computations
                flagMap.erase(role--);                                  // remove the role to reduce future computations TODO:: test with (--role) since loop increments with ++role
            }
        }
    }

/// Removes the occurrence of a role and robot from all fields that involve them
    void Dealer::distribute_remove(DealerDistribute &current, std::vector<std::string> &roleNames, std::vector<RoleScores> &scores,
                                   std::vector<v::RobotView> &unassignedRobots) {
        assert(current.relevant_roleIndices.size() <= scores.size());
        for (auto &i : current.relevant_roleIndices) {     // remove the assigned role from score matrix
            scores.erase(scores.begin() + i);  // remove role from score matrix (column)
            roleNames.erase(roleNames.begin() + i);  // remove role from roleNames vector
            //TODO:: check if the correct scores and roleNames are removed
        }

        for (auto &i : scores) {  // remove the assigned robot from each score matrix column
            for (auto j = current.new_assignments.rbegin(); j != current.new_assignments.rend(); ++j) {
                assert(*j < i.robotScores.size());
                i.robotScores.erase(i.robotScores.begin() + *j);  // remove robot
                //TODO:: check if the score matrix has the correct number of robots (rows)
            }
        }

        for (auto &i : current.new_assignments) {  // remove the assigned robots from all Robots
            unassignedRobots.erase(unassignedRobots.begin() + i);
            //TODO:: check if the correct robots are removed
        }
    }

/// Populates the matrix of robots (row) and roles (column) with scores based on flags and distance
    std::vector<Dealer::RoleScores> Dealer::getScoreMatrix(const std::vector<v::RobotView> &unassignedRobots, const Dealer::FlagMap &flagMap,
                                                           const std::unordered_map<std::string, stp::StpInfo> &stpInfoMap) {
        std::vector<RoleScores> scores;                                         // declare return variable
        scores.reserve(flagMap.size());                                         // reserve space for all roles in the flagMap
        for (auto &i : scores) i.robotScores.reserve(unassignedRobots.size());  // reserve space for all robots in each role score

        for (const auto &[roleName, dealerFlags] : flagMap) {  // loop through all roles in the flagMap
            std::vector<double> role;                          // declare vector to store each robot's score for a role
            role.reserve(unassignedRobots.size());             // reserve space for all robots

            for (auto &robot : unassignedRobots) {                                                            // loop through all robots
                double robotDistanceScore{};                                                                  // declare variable to store a robots distance score
                if (stpInfoMap.find(roleName) != stpInfoMao.end()) {                                          // if the roleName is in stpInfoMap
                    robotDistanceScore = getRobotScoreForDistance(stpInfoMap.find(roleName)->second, robot);  // get the distance score
                }
                auto robotScore = getRobotScoreForRole(dealerFlags.flags, robot);  // get robot score based on the role's dealerFlags

                role.push_back((robotDistanceScore + robotScore.sumScore) / (1 + robotScore.sumWeights));  // simple normalizer formula (robotDistanceScore weight = 1)
            }
            scores.push_back({role, dealerFlags.priority});  // add role scores and priority to return variable
        }
        return scores;
    }

    // Calculate the score for all flags for a role for one robot
    Dealer::RobotRoleScore Dealer::getRobotScoreForRole(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot) {
        double robotScore = 0;
        double sumWeights = 0;
        for (auto flag : dealerFlags) {
            FlagScore ScoreForFlag = getRobotScoreForFlag(robot, flag);
            robotScore += ScoreForFlag.score;
            sumWeights += ScoreForFlag.weight;
        }
        return {robotScore,sumWeights};  // [score,sum of weights]
    }

// Get the score of one flag for a role for one robot
    Dealer::FlagScore Dealer::getRobotScoreForFlag(v::RobotView robot, Dealer::DealerFlag flag) {
        double factor = getWeightForPriority(flag.priority);
        return {factor * getDefaultFlagScores(robot, flag),factor}; // [score,weight]
    }

// Get the distance score for a robot to a position when there is a position that role needs to go to
    double Dealer::getRobotScoreForDistance(const stp::StpInfo &stpInfo, const v::RobotView &robot) {
        double distance{};
        if (robot->getId() == GameStateManager::getCurrentGameState().keeperId) {
            distance = 0;
        } else if (stpInfo.getPositionToMoveTo().has_value()) {
            distance = robot->getPos().dist(stpInfo.getPositionToMoveTo().value());
        } else if (stpInfo.getEnemyRobot().has_value()) {
            distance = robot->getPos().dist(stpInfo.getEnemyRobot().value()->getPos());
        }

        return costForDistance(distance, field->getFieldWidth(), field->getFieldLength());
    }

// TODO these values need to be tuned.
    double Dealer::getWeightForPriority(const DealerFlagPriority &flagPriority) {
        switch (flagPriority) {
            case DealerFlagPriority::LOW_PRIORITY:
                return 0.5;
            case DealerFlagPriority::MEDIUM_PRIORITY:
                return 1;
            case DealerFlagPriority::HIGH_PRIORITY:
                return 5;
            case DealerFlagPriority::REQUIRED:
                return 100;
            default:
                RTT_WARNING("Unhandled dealerflag!")
                return 0;
        }
    }

// TODO these values need to be tuned.
    double Dealer::getDefaultFlagScores(const v::RobotView &robot, const Dealer::DealerFlag &flag) {
        auto fieldWidth = field->getFieldWidth();
        auto fieldLength = field->getFieldLength();
        switch (flag.title) {
            case DealerFlagTitle::CLOSE_TO_THEIR_GOAL:
                return costForDistance(FieldComputations::getDistanceToGoal(*field, false, robot->getPos()), fieldWidth, fieldLength);
            case DealerFlagTitle::CLOSE_TO_OUR_GOAL:
                return costForDistance(FieldComputations::getDistanceToGoal(*field, true, robot->getPos()), fieldWidth, fieldLength);
            case DealerFlagTitle::CLOSE_TO_BALL:
                return costForDistance(robot->getDistanceToBall(), fieldWidth, fieldLength);
            case DealerFlagTitle::CLOSE_TO_POSITIONING:
                return costForProperty(true);
            case DealerFlagTitle::WITH_WORKING_BALL_SENSOR:
                return costForProperty(robot->isWorkingBallSensor());
            case DealerFlagTitle::WITH_WORKING_DRIBBLER:
                return costForProperty(robot->isWorkingDribbler());
            case DealerFlagTitle::READY_TO_INTERCEPT_GOAL_SHOT: {
                // get distance to line between ball and goal
                // TODO this method can be improved by choosing a better line for the interception.
                LineSegment lineSegment = {world.getBall()->get()->getPos(), field->getOurGoalCenter()};
                return lineSegment.distanceToLine(robot->getPos());
            }
            case DealerFlagTitle::KEEPER:
                return costForProperty(robot->getId() == GameStateManager::getCurrentGameState().keeperId);
            case DealerFlagTitle::CLOSEST_TO_BALL:
                return costForProperty(robot->getId() == world.getRobotClosestToBall(rtt::world::us)->get()->getId());
        }
        RTT_WARNING("Unhandled dealerflag!")
        return 0;
    }

// Calculate the cost for distance. The further away the target, the higher the cost for that distance.
    double Dealer::costForDistance(double distance, double fieldWidth, double fieldHeight) {
        auto fieldDiagonalLength = sqrt(pow(fieldWidth, 2.0) + pow(fieldHeight, 2.0));
        return distance / fieldDiagonalLength;
    }

    double Dealer::costForProperty(bool property) { return property ? 0.0 : 1.0; }

} // namespace rtt::ai