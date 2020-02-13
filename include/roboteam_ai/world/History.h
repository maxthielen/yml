//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_HISTORY_H
#define ROBOTEAM_AI_HISTORY_H

#include "WorldData.h"

namespace rtt::ai::world {

class History {
   private:
    using WorldDataPtr = std::shared_ptr<WorldData>;

    WorldBuffer worldBuffer;

   public:
    void addWorld(const WorldData &worldData);
    void addWorld(WorldDataPtr &worldDataPtr);
    const WorldData getPreviousWorld(int worldsBack = 1);
};

}  // namespace rtt::ai::world

#endif  // ROBOTEAM_AI_HISTORY_H