//
// Created by emiel on 28-02-20.
//

#ifndef RTT_PUREPURSUIT_H
#define RTT_PUREPURSUIT_H

#include <vector>

#include "roboteam_utils/Vector2.h"

namespace rtt::ai::control {

    class PurePursuit {

        static Vector2 getPursuingPoint(const std::vector<Vector2>& path, const Vector2& position, double pursuitDistance);

        /**
         * @brief Given a path and a point, find the index of the path segment that is closest to the position
         * @param path
         * @param currentPosition
         * @return
         */
        static int getClosestPathSegment(const std::vector<Vector2> &path, const rtt::Vector2 &pos);

    };

}

#endif //RTT_PUREPURSUIT_H
