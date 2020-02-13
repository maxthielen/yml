//
// Created by jessevw on 19.11.19.
//
#include "include/roboteam_ai/treehelpers/BeingPassedToHelper.h"

#include <include/roboteam_ai/conditions/IsBeingPassedTo.h>
#include <include/roboteam_ai/skills/Receive.h>

#include <bt/composites/Selector.h>
#include <bt/composites/Sequence.h>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>

#include "bt/Role.h"
#include "bt/RoleDivider.h"
#include "skills/Attack.h"
#include "skills/gotopos/GoToPos.h"
#include "treeinterp/tactics/DefaultTactic.h"

namespace bt {
BeingPassedToHelper::BeingPassedToHelper() {}

std::shared_ptr<bt::Node> BeingPassedToHelper::createBeingPassedToChecker() {
    auto localbb = std::make_shared<bt::Blackboard>();

    std::shared_ptr<bt::Sequence> sequence = std::make_shared<bt::Sequence>();
    std::shared_ptr<rtt::ai::Receive> receive = std::make_shared<rtt::ai::Receive>("receivernode", localbb);
    std::shared_ptr<rtt::ai::IsBeingPassedTo> beingPassedTo = std::make_shared<rtt::ai::IsBeingPassedTo>("being passed to", localbb);
    sequence->addChild(beingPassedTo);
    sequence->addChild(receive);
    return sequence;
}
}  // namespace bt