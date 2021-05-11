//
// Created by jaro on 30-10-20.
//

#ifndef RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H
#define RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H

#include "include/roboteam_ai/stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
    class BallOnTheirSideGlobalEvaluation : public BaseEvaluation {
    public:
        [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

        const char* getName() override { return "BallOnTheirSideGlobalEvaluation"; }
    };
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLONTHEIRSIDEGLOBALEVALUATION_H