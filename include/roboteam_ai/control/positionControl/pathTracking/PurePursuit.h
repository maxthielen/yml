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
        static Vector2 getPursuingPoint(const std::vector<Vector2>& path, const Vector2& position, double pursuitDistance);

        /**
         * @brief Given a path and a position, find the index of the path segment that is closest to the position
         * @param path The path from which the closest path segment must be found
         * @param position The position for which the closest path segment must be found
         * @return The index of the start node of the closest path segment
         */
        static int getClosestPathSegment(const std::vector<Vector2> &path, const rtt::Vector2 &position);

    };
}

#endif //RTT_PUREPURSUIT_H
