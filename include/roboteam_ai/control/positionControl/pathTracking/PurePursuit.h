//
// Created by emiel on 28-02-20.
//

#ifndef RTT_PUREPURSUIT_H
#define RTT_PUREPURSUIT_H

#include <vector>

#include "roboteam_utils/Vector2.h"

namespace rtt::ai::control {
class PurePursuit {
   public:
    /** TODO update
     * @brief Given a path, a position, and a pursuit distance, it returns the point to which should be driven
     * This point is called the Pure Pursuit Point
     * @param path The path on which to find the Pure Pursuit Point
     * @param position The current position of the agent
     * @param pursuitDistance The distance to look ahead
     * @return The Pure Pursuit Point. The point to which should be driven
     */
    static std::pair<Vector2, int> getPursuingPoint(const std::vector<Vector2>& path, const Vector2& position, double pursuitDistance, bool forceFirstSegment, int robotId);

    /**
     * @brief Given a path and a position, find the index of the path segment that is closest to the position
     * @param path The path from which the closest path segment must be found
     * @param position The position for which the closest path segment must be found
     * @return The index of the start node of the closest path segment
     */
    static int getClosestPathSegment(const std::vector<Vector2>& path, const rtt::Vector2& position);
};
}  // namespace rtt::ai::control

#endif  // RTT_PUREPURSUIT_H
