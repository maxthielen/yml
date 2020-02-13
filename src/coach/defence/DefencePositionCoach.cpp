
//
// Created by rolf on 18-2-19.
//

#include "coach/defence/DefencePositionCoach.h"
#include "control/ControlUtils.h"
#include "utilities/RobotDealer.h"
#include "world/Field.h"
#include "world/FieldComputations.h"

/// This is a class that computes useful lines and positions for computing defender positions
namespace rtt::ai::coach {

using util = control::ControlUtils;

DefencePositionCoach g_defensivePositionCoach;

bool DefenderBot::validPosition(const world::WorldData &world) {
    for (const auto &bot : world.us) {
        if ((bot->pos - targetPos).length() < 2 * Constants::ROBOT_RADIUS()) {
            return false;
        }
    }
    return true;
}

const world::Robot::RobotPtr DefenderBot::toRobot() {
    world::Robot::RobotPtr robot = std::make_shared<world::Robot>(world::Robot());
    robot->id = -1;
    robot->pos = targetPos;
    robot->angle = orientation;
    return robot;
}
// pick position on the line depending on how aggressive we want to play. aggression factor 1 is very in your face, whilst 0 is as close as possible to the goal
Vector2 DefencePositionCoach::getPosOnLine(const Line &line, double aggressionFactor) {
    if (aggressionFactor < 0) {
        return line.end;
    } else if (aggressionFactor > 1) {
        return line.start;
    }
    return line.end + (line.start - line.end) * aggressionFactor;
}
// get the direction facing towards the end of the Line
double DefencePositionCoach::getOrientation(const Line &line) { return (line.start - line.end).angle(); }

// computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius
std::shared_ptr<Line> DefencePositionCoach::getBlockLineSegment(const Field &field, const Line &openGoalSegment, const Vector2 &point, double collisionRadius, double margin) {
    if (margin == -1.0) {
        margin = defenceLineMargin;
    }

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos = point + (FurthestBlock - point).stretchToLength(collisionRadius);  // start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (FieldComputations::pointIsInDefenceArea(field, startPos, true, margin)) {
        return nullptr;
    }
    // check intersections with defense area and shorten line if needed
    Line line = shortenLineForDefenseArea(field, startPos, FurthestBlock, margin);
    std::shared_ptr<Line> segment = std::make_shared<Line>(line);
    return segment;
}

/// computes the intersection of the bisector of the angle to OpenGoalsegment from point and the defence area line
std::shared_ptr<Vector2> DefencePositionCoach::blockOnDefenseLine(const Field &field, const Line &openGoalSegment, const Vector2 &point) {
    // margin by which we shift the defence area line forwards
    double margin = defenceLineMargin;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    // compute the bisector
    Vector2 lineToSideOne = openGoalSegment.start - point;
    Vector2 lineToSideTwo = (openGoalSegment.end - point);
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    // if starting point is in the defence area there is no room for a robot to squeeze in
    if (FieldComputations::pointIsInDefenceArea(field, startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo) * 0.5;  // this defines the line on which the bisector lies.
    // now compute intersection with the defense area and return this (if it exists)
    return FieldComputations::lineIntersectionWithDefenceArea(field, true, point, endPos, margin);
}
/// gets the furthest position at which an obstacle will block the entire Angle
/// intuitively you can understand this as the closest point to which a circle of collissionRadius 'fits' in between the two lines
Vector2 DefencePositionCoach::getBlockPoint(const Line &openGoalSegment, const Vector2 &point, double collisionRadius) {
    // compute the bisector of the angle of point and the two ends of the openGoalSegment
    Vector2 lineToSideOne = (openGoalSegment.start - point).normalize();
    Vector2 lineToSideTwo = (openGoalSegment.end - point).normalize();
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo) * 0.5;  // ending point on the bisector, which always just intersects the goalLine
    // compute the furthest distance at which the entire segment is blocked
    double theta = lineToSideOne.angle() - (endPos - point).angle();  // half of the angle of the bisector
    double collisionDist = collisionRadius / sin(theta);
    double angle = (endPos - point).angle();
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate(angle);
    return FurthestBlock;
}

// if the line hits the defence area shorten it, otherwise just return the original line
Line DefencePositionCoach::shortenLineForDefenseArea(const Field &field, const Vector2 &lineStart, const Vector2 &lineEnd, double defenseMargin) {
    Line line;
    std::shared_ptr<Vector2> intersectPos = FieldComputations::lineIntersectionWithDefenceArea(field, true, lineStart, lineEnd, defenseMargin);
    if (!intersectPos) {
        // return the original line
        line = Line(lineStart, lineEnd);
    } else {
        // it intersects with the defence area, so we use the intersect position
        line = Line(lineStart, *intersectPos);
    }
    return line;
}
world::WorldData DefencePositionCoach::removeBotFromWorld(world::WorldData world, int id, bool ourTeam) {
    auto robots = ourTeam ? world.us : world.them;
    robots.erase(std::remove_if(robots.begin(), robots.end(), [id](world::Robot::RobotPtr robot) { return robot->id == id; }));
    ourTeam ? world.us = robots : world.them = robots;
    return world;
}
Vector2 DefencePositionCoach::getMostDangerousPos(const world::WorldData &world) {
    if (world.ball->getVel().length() > 0.5) {
        return world.ball->getPos() + world.ball->getVel() * 0.3;
    }
    return world.ball->getPos();
}
std::vector<std::pair<PossiblePass, double>> DefencePositionCoach::createPassesAndDanger(const Field &field, const world::WorldData &world) {
    std::vector<std::pair<PossiblePass, double>> passWithScore;
    // check the passes from the robot towards every other bot and calculate their danger
    for (const auto &theirBot : world.them) {
        // TODO: perhaps ignore robots we have already covered here. The score should be gutted regardless.
        PossiblePass pass(*theirBot, world.ball->getPos());
        double danger = pass.score(field, world);  // check how dangerous the pass is in our simulated world
        std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
        passWithScore.push_back(passPair);
    }
    return passWithScore;
}
std::vector<PossiblePass> DefencePositionCoach::sortPassesByDanger(std::vector<std::pair<PossiblePass, double>> &passWithScore) {
    // order passes from most dangerous to least
    std::sort(passWithScore.begin(), passWithScore.end(), [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) { return left.second > right.second; });
    std::vector<PossiblePass> passes;
    for (const auto &passScore : passWithScore) {
        passes.push_back(passScore.first);
    }
    return passes;
}
std::vector<PossiblePass> DefencePositionCoach::createPassesSortedByDanger(const Field &field, const rtt::ai::world::WorldData &world) {
    auto passes = createPassesAndDanger(field, world);
    return sortPassesByDanger(passes);
}
DefenderBot DefencePositionCoach::createBlockToGoal(const Field &field, const PossiblePass &pass, double aggressionFactor, const Line &blockLine) {
    DefenderBot bot;
    bot.type = botType::BLOCKTOGOAL;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = getPosOnLine(blockLine, aggressionFactor);
    bot.orientation = getOrientation(blockLine);
    return bot;
}
DefenderBot DefencePositionCoach::createBlockToGoal(const Field &field, const PossiblePass &pass, const Vector2 &position, const Line &blockLine) {
    DefenderBot bot;
    bot.type = botType::BLOCKTOGOAL;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = position;
    bot.orientation = getOrientation(blockLine);
    return bot;
}
DefenderBot DefencePositionCoach::createBlockOnLine(const Field &field, const PossiblePass &pass, const Vector2 &blockPos) {
    DefenderBot bot;
    bot.type = botType::BLOCKONLINE;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = blockPos;
    // draw a line for easy orientation computation
    Line line = Line(pass.endPos, blockPos);
    bot.orientation = getOrientation(line);
    return bot;
}
DefenderBot DefencePositionCoach::createBlockPass(const Field &field, PossiblePass &pass, const Vector2 &blockPoint) {
    DefenderBot bot;
    bot.type = botType::BLOCKPASS;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = blockPoint;
    bot.orientation = pass.faceLine();
    return bot;
}
std::shared_ptr<Vector2> DefencePositionCoach::blockOnDefenseAreaLine(const Field &field, const PossiblePass &pass, const world::WorldData &world) {
    auto visibleParts = FieldComputations::getVisiblePartsOfGoal(field, true, pass.endPos, world);
    // get the largest segment
    std::sort(visibleParts.begin(), visibleParts.end(), [](const Line &a, const Line &b) { return abs(a.end.y - a.start.y) > abs(b.end.y - b.start.y); });
    if (!visibleParts.empty()) {
        return blockOnDefenseLine(field, visibleParts[0], pass.endPos);
    }
    return nullptr;
}
/// checks for a given pass in a simulatedWorld if we can block it's receiver shot to goal and returns a line on which to stand if this is the case
std::shared_ptr<Line> DefencePositionCoach::blockToGoalLine(const Field &field, const PossiblePass &pass, const world::WorldData &world) {
    // get the blockLine segment from the ending position of the pass
    auto visibleParts = FieldComputations::getVisiblePartsOfGoal(field, true, pass.endPos, world);
    // get the largest segment (sort by size)
    std::sort(visibleParts.begin(), visibleParts.end(), [](const Line &a, const Line &b) { return abs(a.end.y - a.start.y) > abs(b.end.y - b.start.y); });
    if (!visibleParts.empty()) {
        auto blockLine = getBlockLineSegment(field, visibleParts[0], pass.endPos);
        return blockLine;
    }
    return nullptr;
}

/// searches the most dangerous position and then gets the segment which blocks that (if it exists/is possible)
std::shared_ptr<Line> DefencePositionCoach::blockBallLine(const Field &field, const world::WorldData &world) {
    if (world.ball) {
        Vector2 mostDangerousPos = getMostDangerousPos(world);
        if (FieldComputations::pointIsInField(field, mostDangerousPos, -0.1)) {
            return getBlockLineSegment(field, FieldComputations::getGoalSides(field, true), mostDangerousPos);
        }
    }
    return nullptr;
}

DefenderBot DefencePositionCoach::createBlockBall(const Field &field, const Line &blockLine) {
    // TODO: handle special handling for the case where we can't block using this bot (e.g. try to make keeper actively block?)
    DefenderBot bot;
    bot.type = botType::BLOCKBALL;
    bot.targetPos = findPositionForBlockBall(field, blockLine);
    bot.blockFromID = world::world->whichRobotHasBall(THEIR_ROBOTS) ? (world::world->whichRobotHasBall(THEIR_ROBOTS)->id) : (-1);
    bot.orientation = getOrientation(blockLine);
    return bot;
}
Vector2 DefencePositionCoach::findPositionForBlockBall(const Field &field, const Line &blockLine) {
    // if the blocking position is way away from our goal keep the robot on our side
    double maxForwardLineX = maxX(field);
    Vector2 position = getPosOnLine(blockLine, 0.1);
    if (position.x > maxForwardLineX) {
        double fieldWidth = field.getFieldWidth();
        Vector2 bottomLine(maxForwardLineX, -fieldWidth * 0.5);
        Vector2 topLine(maxForwardLineX, fieldWidth * 0.5);
        Vector2 intersect = control::ControlUtils::twoLineIntersection(blockLine.start, blockLine.end, bottomLine, topLine);
        return intersect;
    }
    return position;
}
double DefencePositionCoach::maxX(const Field &field) { return field.getFieldLength() / 10.0 * -1.0; }

world::WorldData DefencePositionCoach::getTheirAttackers(const Field &field, const world::WorldData &world) {
    std::vector<world::Robot::RobotPtr> theirAttackers;
    for (auto &robot : world.them) {
        // we remove any attackers that are outside of the field or in our defence area
        if (!FieldComputations::pointIsInDefenceArea(field, robot->pos, true, 0.04) && FieldComputations::pointIsInField(field, robot->pos, -0.1)) {
            theirAttackers.push_back(robot);
        }
    }
    world::WorldData newWorld = world;
    newWorld.them = theirAttackers;
    return newWorld;
}
bool DefencePositionCoach::validNewPosition(const Field &field, const Vector2 &position, const world::WorldData &world) {
    if (position.x > maxX(field)) {
        return false;
    }
    double collisionRadius = calculationCollisionRad;  // a little smaller than 2 robot radii so that we can make solid walls still
    for (const auto &robot : world.us) {
        if ((robot->pos - position).length() < collisionRadius) {
            return false;
        }
    }
    return true;
}
std::shared_ptr<double> DefencePositionCoach::pickNewPosition(const Field &field, const Line &line, const world::WorldData &world) {
    // search a position on the line on which we can position.
    for (int aggresionFactor = 0; aggresionFactor <= searchPoints; ++aggresionFactor) {
        if (validNewPosition(field, getPosOnLine(line, aggresionFactor / searchPoints), world)) {
            std::shared_ptr<double> point = std::make_shared<double>(aggresionFactor);
            return point;
        }
    }
    // the whole line is completely blocked (atleast on the points we tested)
    return nullptr;
}

std::shared_ptr<Vector2> DefencePositionCoach::pickNewPosition(const Field &field, PossiblePass pass, const world::WorldData &world) {
    std::shared_ptr<Vector2> point = nullptr;
    double segments = 30.0;
    // we pick new points on which we can defend preferably as close as possible to the middle of the pass.
    for (int j = 0; j <= segments * 0.5; ++j) {
        Vector2 forwardPosition = pass.posOnLine(0.5 + j / segments);
        if (validNewPosition(field, forwardPosition, world) && !FieldComputations::pointIsInDefenceArea(field, forwardPosition, true, defenceLineMargin)) {
            return std::make_shared<Vector2>(forwardPosition);
        }
        Vector2 backwardPosition = pass.posOnLine(0.5 - j / segments);
        if (validNewPosition(field, backwardPosition, world) && !FieldComputations::pointIsInDefenceArea(field, forwardPosition, true, defenceLineMargin)) {
            return std::make_shared<Vector2>(backwardPosition);
        }
    }
    return nullptr;
}

world::WorldData DefencePositionCoach::setupSimulatedWorld(const Field &field) {
    world::WorldData sWorld = world::world->getWorld();
    sWorld.us.clear();
    sWorld = getTheirAttackers(field, sWorld);  // we select only the relevant robots
    return sWorld;
}
std::shared_ptr<DefenderBot> DefencePositionCoach::blockMostDangerousPos(const Field &field) {
    // block the most dangerous position to the goal completely if possible.
    // check if it is possible to find a position that is legal
    auto crucialBlock = blockBallLine(field, simulatedWorld);
    if (crucialBlock) {
        DefenderBot crucialDefender = createBlockBall(field, *crucialBlock);  // if so, create a defender
        return std::make_shared<DefenderBot>(crucialDefender);
    }
    return nullptr;
}

std::shared_ptr<DefenderBot> DefencePositionCoach::blockPass(const Field &field, PossiblePass pass) {
    // this function tries multiple ways to block a pass. Returns true if it succeeded, false if it fails.
    // first try blocking the goal vision of the robot being passed to
    auto blockLine = blockToGoalLine(field, pass, simulatedWorld);
    if (blockLine) {
        // try to find a position on the line that is valid
        auto aggressionFactor = pickNewPosition(field, *blockLine, simulatedWorld);
        if (aggressionFactor) {
            DefenderBot defender = createBlockToGoal(field, pass, *aggressionFactor, *blockLine);
            return std::make_shared<DefenderBot>(defender);
        }
        // try putting it on the defence Line instead (as the robot is very likely far away
        double fieldWidth = field.getFieldWidth();

        // Floating point errors sigh (hence the -0.0001)
        Vector2 bottomLine(maxX(field) - 0.0001, -fieldWidth * 0.5);
        Vector2 topLine(maxX(field) - 0.0001, fieldWidth * 0.5);
        Vector2 intersectPos = control::ControlUtils::twoLineIntersection(blockLine->start, blockLine->end, bottomLine, topLine);
        if (validNewPosition(field, intersectPos, simulatedWorld)) {
            DefenderBot defender = createBlockToGoal(field, pass, intersectPos, *blockLine);
            return std::make_shared<DefenderBot>(defender);
        }
    }
    // then try blocking on the defense line (closer to the line) if that is possible
    auto blockPos = blockOnDefenseAreaLine(field, pass, simulatedWorld);
    if (blockPos) {
        if (validNewPosition(field, *blockPos, simulatedWorld)) {
            DefenderBot defender = createBlockOnLine(field, pass, *blockPos);
            return std::make_shared<DefenderBot>(defender);
        }
    }
    // then try to intercept the pass, we try to find a spot along the pass where we can stand
    auto passBlock = pickNewPosition(field, pass, simulatedWorld);
    if (passBlock) {
        DefenderBot defender = createBlockPass(field, pass, *passBlock);
        return std::make_shared<DefenderBot>(defender);
    }
    return nullptr;
}
// if we add a defender we want to both add it to simulation and our stored defender array for return
void DefencePositionCoach::addDefender(DefenderBot defender) {
    simulatedWorld.us.push_back(defender.toRobot());
    defenders.push_back(defender);
}

std::vector<DefenderBot> DefencePositionCoach::decidePositions(const Field &field, const std::vector<DefenderBot> &lockedDefenders, std::vector<int> freeRobots) {
    std::vector<DefenderBot> oldDefenders = defenders;
    defenders.clear();  // we are recomputing the positions again
    int defenderAmount = lockedDefenders.size() + freeRobots.size();
    if (defenderAmount <= 0) {
        return defenders;
    }  // we don't actually need to calculate now.
    simulatedWorld = setupSimulatedWorld(field);
    // handle all the locked robots
    std::tuple<bool, int, std::vector<int>> temp = decideLockedPositions(field, lockedDefenders, freeRobots);

    bool blockedMostDangerousPos = std::get<0>(temp);
    int lockedCount = std::get<1>(temp);
    freeRobots = std::get<2>(temp);
    // now we only have free robots left;
    if (!blockedMostDangerousPos && defenders.size() < defenderAmount) {
        auto bot = blockMostDangerousPos(field);  // first we handle the most dangerous position first
        if (bot) {
            addDefender(*bot);
        }
    }
    // for the remainder we look at the possiblePasses and block the most dangerous bots
    std::vector<PossiblePass> passes = createPassesSortedByDanger(field, simulatedWorld);
    while ((defenders.size()) != defenderAmount && !passes.empty()) {
        auto foundNewDefender = blockPass(field, passes[0]);  // we try to cover the most dangerous pass in multiple ways
        // if we find a defender we need to recalculate the danger of our passes to reflect the new robot.
        if (!foundNewDefender) {
            // if we cannot find a way to cover it, we remove the attacker from the simulated world (otherwise we get 'stuck')
            simulatedWorld = removeBotFromWorld(simulatedWorld, passes[0].toBot.id, false);
            // this should pretty much never happen.
            std::cerr << "Pass to robot" << passes[0].toBot.id << " removed in defensiveCoach!" << std::endl;
        } else {
            addDefender(*foundNewDefender);
        }
        passes = createPassesSortedByDanger(field, simulatedWorld);  // recalculate the danger after the new position
    }
    // TODO: find a procedure if we cannot really cover all of the robots off, e.g. default positions or such
    assignIDs(lockedCount, freeRobots, oldDefenders);  // divide the ID's of the last robots over the remaining available ID's.
    return defenders;
}

std::tuple<bool, int, std::vector<int>> DefencePositionCoach::decideLockedPositions(const Field &field, const std::vector<rtt::ai::coach::DefenderBot> &lockedDefenders,
                                                                                    std::vector<int> freeRobots) {
    bool blockedMostDangerousPos = false;
    int lockedCount = 0;
    std::vector<PossiblePass> passes = createPassesSortedByDanger(field, simulatedWorld);
    for (const auto &lockedDefender : lockedDefenders) {
        bool replacedDefender = false;
        if (lockedDefender.type != BLOCKBALL) {
            for (const auto &pass : passes) {
                if (pass.toBot.id == lockedDefender.blockFromID) {
                    auto newDefender = blockPass(field, pass);
                    if (newDefender) {
                        lockedCount++;
                        newDefender->id = lockedDefender.id;
                        newDefender->coveredCount = lockedDefender.coveredCount + 1;
                        addDefender(*newDefender);
                        replacedDefender = true;
                    }
                    break;
                }
            }
        }
        // we need special handling for if the robot is blocking the most dangerous position
        else {
            auto newDefender = blockMostDangerousPos(field);
            if (newDefender) {
                lockedCount++;
                newDefender->id = lockedDefender.id;
                newDefender->coveredCount = lockedDefender.coveredCount + 1;
                addDefender(*newDefender);
                blockedMostDangerousPos = true;
                replacedDefender = true;
            }
        }
        if (!replacedDefender) {
            freeRobots.push_back(lockedDefender.id);  // if we somehow cannot cover this robot anymore, we set it to free
        }
    }
    return std::make_tuple(blockedMostDangerousPos, lockedCount, freeRobots);
}
// the following algorithm takes the closest robot for each available defender to decide which robot goes where.
// Since the points are ordered on priority from the above algorithm the most important points come first
// It might be better to use an algorithm that is more complicated (e.g. hungarian) but then we might need some kind of system which gives the first points more 'priority'
void DefencePositionCoach::assignIDs(int lockedCount, std::vector<int> freeRobotIDs, const std::vector<DefenderBot> &oldDefenders) {
    std::vector<int> freeIDs = freeRobotIDs;
    for (int j = lockedCount; j < defenders.size(); ++j) {
        int closestId = -1;
        auto closestDist = 9e9;
        for (int botId : freeIDs) {
            auto bot = world::world->getRobotForId(botId, true);
            if (bot) {
                if ((defenders[j].targetPos - bot->pos).length() < closestDist) {
                    closestId = botId;
                    closestDist = (defenders[j].targetPos - bot->pos).length();
                }
            }
        }
        if (closestId != -1) {
            defenders[j].id = closestId;
            freeIDs.erase(std::find(freeIDs.begin(), freeIDs.end(), closestId));
            for (const auto &oldDefender : oldDefenders) {
                // if the robot is still covering the same target.
                if (oldDefender.id == closestId && oldDefender.blockFromID == defenders[j].blockFromID) {
                    defenders[j].coveredCount = oldDefender.coveredCount + 1;
                    break;
                }
            }
        }
    }
}
}  // namespace rtt::ai::coach