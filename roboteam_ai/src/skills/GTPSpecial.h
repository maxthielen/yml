//
// Created by baris on 15-1-19.
//

#ifndef ROBOTEAM_AI_BASICGOTOPOS_H
#define ROBOTEAM_AI_BASICGOTOPOS_H

#include <roboteam_ai/src/control/PositionController.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include <roboteam_ai/src/coach/Ballplacement.h>
#include <roboteam_ai/src/utilities/Field.h>
#include "Skill.h"
#include "GoToPos.h"
#include "../interface/drawer.h"


namespace rtt {
namespace ai {

class GTPSpecial : public GoToPos {
    private:
        enum Type {
            goToBall,
            ballPlacementBefore,
            ballPlacementAfter,
            getBallFromSide,
            defaultType
        };

        Type type;
        Type stringToType(std::string string);

    public:
        explicit GTPSpecial(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;

        double getballFromSideMargin = 0.3;
        Vector2 getBallFromSideLocation();

};
}
}

#endif //ROBOTEAM_AI_BASICGOTOPOS_H