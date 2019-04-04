//
// Created by robzelluf on 4/4/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/control/ControlUtils.h>

TEST(CoachTest, offensive_coach_test) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    rtt::ai::Field::set_field(field);

    std::vector<Vector2> offensivePositions = rtt::ai::coach::g_offensiveCoach.getNewOffensivePositions();

    int counter = 0;
    std::vector<Vector2> newOffensivePositions;

    // Calculate the next offensive positions 10 times
    while (counter < 10) {
        newOffensivePositions = rtt::ai::coach::g_offensiveCoach.getNewOffensivePositions();
        counter++;
    }

    for(int i = 0; i < offensivePositions.size(); i++) {
        if (offensivePositions.at(i) == newOffensivePositions.at(i)) {
            ASSERT_TRUE(false);
        }
    }
}