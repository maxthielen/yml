//
// Created by emiel on 28-02-20.
//

#include "control/positionControl/pathTracking/PurePursuit.h"

#include <algorithm>
#include "roboteam_utils/LineSegment.h"
#include "interface/api/Input.h"
namespace rtt::ai::control {

std::pair<Vector2, int> PurePursuit::getPursuingPoint(const std::vector<Vector2> &path, const rtt::Vector2 &position, double pursuitDistance, bool forceFirstSegment, int robotId) {
    if (path.size() < 2) {
        std::cerr << "[PurePursuit::getPursuingPoint] Trying to get a pursuit point for a path without segments!" << std::endl;
        return {position, 1};
    }

    double _pursuitDistance = pursuitDistance;

    /* First, find the segment to which the position is the closest */
    /* If forceFirstSegment is true, than the first segment is used from here on out */
    int iNodeClosest = 0;
    if(!forceFirstSegment)
        iNodeClosest = getClosestPathSegment(path, position);

    /* Get the segment on which the pursuit point has to be found */
    Vector2 segmentStart = path.at(iNodeClosest);
    Vector2 segmentStop = path.at(iNodeClosest + 1);
    LineSegment segment(segmentStart, segmentStop);

    /* Check if the segment is further away than the pursuit distance */
    /* If it is not, drive towards the segment in the shortest way possible */
    Vector2 projectionOnClosestSegment = segment.project(position);
    double distanceToClosestSegment = (projectionOnClosestSegment - position).length();
    if(pursuitDistance <= distanceToClosestSegment){
        Vector2 pursuitPosition = position + (projectionOnClosestSegment - position).stretchToLength(pursuitDistance);
        return {pursuitPosition, iNodeClosest};
    }

    /* Subtract the distance between the position and the segment, from the pursuit distance */
    pursuitDistance -= distanceToClosestSegment;

    /** Note : the following is needed because, in the current implementation of numtrees,
     * path segments with a length 0 are sometimes given. (startSegment == stopSegment)
     */
    /* If the closest line segment has no length, all further calculations are invalid. For example, segment.project will return NaN.
     * Simply return the start of the segment.
     */
    if (segment.length() == 0) {
        std::cerr << "[PurePursuit::getPursuingPoint] Path given which contains a segment with length 0!" << std::endl;
        Vector2 pursuitPosition = position + (segmentStart - position).stretchToLength(_pursuitDistance);
        interface::Input::drawData(interface::Visual::PATHFINDING, {segmentStart, segmentStop}, Qt::red, robotId, interface::Drawing::CROSSES);
        return {pursuitPosition, iNodeClosest};
    }

    /* Project the current location onto the segment */
    Vector2 projectionOnSegment = segment.project(position);

    /* If the pursuit distance is larger than the remaining distance on the current segment, either:
     * - If there is no next segment, return the end of the current segment
     * - If there is a next segment, move over to the next segment.
     *     - The remaining pursuit distance might be larger than this segment as well. If so, drive to the end of this segment
     *       Don't move forward more than one segment. This would mean skipping segments, which cause shortcuts and possibly collisions
     */
    /* Calculate how much distance is left between the projection on the segment and the end of the segment */
    double remainingDistance = (segmentStop - projectionOnSegment).length();
    if (remainingDistance < pursuitDistance) {
        // If there is no next segment, return the stop position of the current segment
        if (iNodeClosest == path.size() - 2)
            return {segmentStop, iNodeClosest};

        /* Set the next segment */
        iNodeClosest++;
        segmentStart = path.at(iNodeClosest );
        segmentStop = path.at(iNodeClosest + 1);
        segment = LineSegment(segmentStart, segmentStop);
        /* Subtract the remaining distance from the pursuit distance */
        pursuitDistance -= remainingDistance;
        /* Make sure that the pursuit distance isn't larger than the now current segment. This prevents overshooting */
        pursuitDistance = std::max(pursuitDistance, segment.length());
        /* Move the pursuit point from the start of the segment to the end of the segment */
        projectionOnSegment = segmentStart;
    }

    /* Move the pursuit point from the projected location towards the end of the segment */
    Vector2 pursuitPoint = projectionOnSegment + (segmentStop - projectionOnSegment).stretchToLength(pursuitDistance);

    return {pursuitPoint, iNodeClosest};
}

int PurePursuit::getClosestPathSegment(const std::vector<Vector2> &path, const rtt::Vector2 &position) {
    if (path.size() < 2) {
        std::cerr << "[PurePursuit::getClosestPathSegment] Trying to get a pursuit point for a path without segments!" << std::endl;
        return 0;
    }

    double closestDistance = 1e9;
    // Assume that the first node is closest. Ensures a valid return value
    int iNodeClosest = 0;

    // For each segment in the path
    for (int iNode = 0; iNode < path.size() - 1; iNode++) {
        // Get the segment from the path
        LineSegment segment(path.at(iNode), path.at(iNode + 1));
        // Segments with zero distance are invalid. segment.distanceToLine will return -nan. Skip the invalid segment
        if (segment.length() == 0.0) continue;

        // Calculate the shortest distance between this segment and the given position
        double distance = segment.distanceToLine(position);
        // If this segment is currently the closest segment to the position, store it
        if (distance <= closestDistance) {
            closestDistance = distance;
            iNodeClosest = iNode;
        }
    }

    return iNodeClosest;
}
}  // namespace rtt::ai::control