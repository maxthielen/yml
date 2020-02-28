//
// Created by emiel on 28-02-20.
//

#include "control/positionControl/pathTracking/PurePursuit.h"

#include <algorithm>
#include "roboteam_utils/LineSegment.h"
#include <cmath>

namespace rtt::ai::control {

    Vector2 PurePursuit::getPursuingPoint(const std::vector<Vector2> &path, const rtt::Vector2 &position, double pursuitDistance) {
        if(path.size() < 2){
            std::cerr << "[PurePursuit::getPursuingPoint] Trying to get a pursuit point for a path without segments!" << std::endl;
            return position;
        }

        /* First, find the segment to which the position is the closest */
        int iNodeClosest = getClosestPathSegment(path, position);


        /** Note : The following is needed because, in the current implementation of numtrees,
         * the path segment that is currently being driven is not included in the path.
         */
        /* Subtract the distance to the start of the closest segment from the pursuit distance */
        pursuitDistance -= (position - path.at(iNodeClosest)).length();
        /* Make sure that the pursuit distance is not negative */
        pursuitDistance = std::max(0., pursuitDistance);

        /* Get the line segment on which the pursuit point has to be found */
        Vector2 segmentStart = path.at(iNodeClosest);
        Vector2 segmentStop  = path.at(iNodeClosest+1);
        LineSegment segment(segmentStart, segmentStop);

        /** Note : the following is needed because, in the current implementation of numtrees,
         * path segments are given with a length 0. (startSegment == stopSegment)
         */
        /* If the closest line segment has no length, all further calculations are invalid. For example, segment.project will return NaN.
         * Simply return the start of the segment.
         */
        if(segment.length() == 0)
            return segmentStart;

        /* Project the current location onto the segment */
        Vector2 positionOnSegment = segment.project(position);

        /* If the pursuit distance is larger than the remaining distance on the current segment, either:
         * - If there is no next segment, return the end of the current segment
         * - If there is a next segment, move over to the next segment.
         *     - The remaining pursuit distance might be larger than this segment as well. If so, drive to the end of this segment
         *       Don't move forward more than one segment. This means skipping segments, which cause shortcuts and possibly collisions
         */
        /* Calculate how much distance is left between the point on the segment and the end of the segment */
        double remainingDistance = (segmentStop - positionOnSegment).length();
        if(remainingDistance < pursuitDistance){
            // If there is no next segment, return the stop position of the current segment
            if(iNodeClosest == path.size() - 2)
                return segmentStop;

            /* Set the next segment */
            segmentStart = path.at(iNodeClosest+1);
            segmentStop  = path.at(iNodeClosest+2);
            segment = LineSegment(segmentStart, segmentStop);
            /* Subtract the remaining distance from the pursuit distance */
            pursuitDistance -= remainingDistance;
            /* Make sure that the pursuit distance isn't larger than the now current segment. This prevents overshooting */
            pursuitDistance = std::max(pursuitDistance, segment.length());
            /* Move the pursuit point from the start of the segment to the end of the segment */
            positionOnSegment = segmentStart;
        }

        /* Move the pursuit point from the projected location towards the end of the segment */
        Vector2 pursuitPoint = positionOnSegment + (segmentStop - positionOnSegment).stretchToLength(pursuitDistance);

        return pursuitPoint;
    }

    int PurePursuit::getClosestPathSegment(const std::vector<Vector2> &path, const rtt::Vector2 &position){

        if(path.size() < 2){
            std::cerr << "[PurePursuit::getClosestPathSegment] Trying to get a pursuit point for a path without segments!" << std::endl;
            return 0;
        }

        double closestDistance = 1e9;
        // Assume that the first node is closest. Ensures a valid return value
        int iNodeClosest = 0;

        // For each segment in the path
        for(int iNode = 0; iNode < path.size() - 1; iNode++) {
            // Get the segment from the path
            LineSegment segment(path.at(iNode), path.at(iNode + 1));
            // Segments with zero distance are invalid. segment.distanceToLine will return -nan. Skip the invalid segment
            if(segment.length() == 0.0)
                continue;

            // Calculate the shortest distance between this segment and the given position
            double distance = segment.distanceToLine(position);
            // If this segment is currently the closest segment to the position, store it
            if(distance <= closestDistance){
                closestDistance = distance;
                iNodeClosest = iNode;
            }
        }

        return iNodeClosest;
    }
}