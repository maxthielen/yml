//
// Created by luukkn on 21-04-20.
//

#ifndef RTT_WEHAVEMAJORITYINVARIANT_H
#define RTT_WEHAVEMAJORITYINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class WeHaveMajorityInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "WeHaveMajority"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_WEHAVEMAJORITYINVARIANT_H