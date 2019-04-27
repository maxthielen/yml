//
// Created by rolf on 3-4-19.
//

#include "DefenceDealer.h"
#include "roboteam_ai/src/interface/drawer.h"

namespace rtt {
namespace ai {
namespace coach {

DefenceDealer g_DefenceDealer;

/// adds a defender to the available defendersList
void DefenceDealer::addDefender(int id) {
    bool robotIsRegistered = std::find(defenders.begin(), defenders.end(), id) != defenders.end();
    if (! robotIsRegistered) {
        defenders.push_back(id);
        //std::cout << "registered defender id:" << id << std::endl;
    }
    else {
        //std::cerr << "Defender is already registered, check your tree!!" << std::endl;
    }
}

/// get the specific position of a defender with specified id
std::shared_ptr<std::pair<Vector2, double>> DefenceDealer::getDefenderPosition(int id) {
    auto element = defenderLocations.find(id);
    if (element == defenderLocations.end()) {
        return nullptr;
    }
    else return std::make_shared<std::pair<Vector2, double>>(defenderLocations[id]);
}
void DefenceDealer::visualizePoints() {
    int i = 0;
    std::vector<std::pair<Vector2, QColor>> vis2;
    for (const auto &location : defenderLocations) {
        std::vector<QColor> colors = {Qt::green, Qt::red, Qt::blue, Qt::darkYellow, Qt::darkMagenta, Qt::cyan};
        std::pair<Vector2, QColor> pair = std::make_pair(location.second.first, colors[i%colors.size()]);
        vis2.emplace_back(pair);
        i ++;
    }
    ai::interface::Drawer::setTestPoints(vis2);
}
/// calculates the defender locations for all available defenders
void DefenceDealer::updateDefenderLocations() {
        // clear the defenderLocations
        defenderLocations.clear();
        std::vector<int> availableDefenders = defenders;
        defenders.clear();
        // decide the locations to defend
        std::vector<DefencePositionCoach::DefenderBot> defenderBots = g_defensivePositionCoach.decidePositions(
                availableDefenders.size());
        // the following algorithm takes the closest robot for each available defender to decide which robot goes where.
        // Since the points are ordered on priority from the above algorithm the most important points come first
        // It might be better to use an algorithm that is more complicated (e.g. hungarian) but then we might need some kind of system which gives the first points more 'priority'
        for (const auto &defenderBot : defenderBots) {
            int closestId = - 1;
            double closestDist = DBL_MAX;
            for (int botId : availableDefenders) {
                auto bot = world::world->getRobotForId(botId, true);
                if (bot) {
                    if ((defenderBot.targetPos - bot->pos).length() < closestDist) {
                        closestId = botId;
                        closestDist = (defenderBot.targetPos - bot->pos).length();
                    }
                }
                else {
                    std::cerr << "Could not find robot " << botId << " to defend!";
                }
            }
            if (closestId != - 1) {
                defenderLocations[closestId] = {defenderBot.targetPos, defenderBot.orientation};
                availableDefenders.erase(std::find(availableDefenders.begin(), availableDefenders.end(), closestId));
            }
            else {
                std::cerr << "Could not find a robot to defend location!!!";
                return;
            }
        }
        //visualization
        visualizePoints();
}
}//coach
}//ai
}//rtt