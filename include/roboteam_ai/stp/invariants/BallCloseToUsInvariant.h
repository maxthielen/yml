//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLCLOSETOUSINVARIANT_H
#define RTT_BALLCLOSETOUSINVARIANT_H

#include <NFParam/Param.h>

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallCloseToUsInvariant : public BaseInvariant {
   public:
    BallCloseToUsInvariant() noexcept;

    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallCloseToUs"; }

   private:
    /**
     * Calculates the actual metric value using the piecewise linear function member
     * @param x the x of the function
     * @return metric value between 0-255
     */
    [[nodiscard]] uint8_t calculateMetric(const double& x) const noexcept;

    /**
     * Unique pointer to the piecewise linear function that calculates the fuzzy value
     */
    std::unique_ptr<nativeformat::param::Param> piecewiseLinearFunction;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLCLOSETOUSINVARIANT_H