#include <gtest/gtest.h>
#include <cmath>
#include "../include/utils.hpp"
#include "../include/constant.hpp"
const float EPSILON = 1e-6;

TEST(AngMod2PITest, HandlesZero) {
    EXPECT_NEAR(AngMod2PI(0.F), 0.0F, EPSILON);
}

TEST(AngMod2PITest, HandlesNegativeAngles) {
    EXPECT_NEAR(AngMod2PI(-M_PI_F), M_PI_F, EPSILON);
    EXPECT_NEAR(AngMod2PI(-2.0F * M_PI_F), 0.0F, EPSILON);
    EXPECT_NEAR(AngMod2PI(-3.0F * M_PI_F), M_PI_F, EPSILON);
}

TEST(AngMod2PITest, HandlesPositiveAngles) {
    EXPECT_NEAR(AngMod2PI(2.0F * M_PI_F), 2.0F * M_PI_F, EPSILON);
    EXPECT_NEAR(AngMod2PI(4.0F * M_PI_F), 2.0F * M_PI_F, EPSILON);
    EXPECT_NEAR(AngMod2PI(3.0F * M_PI_F), M_PI_F, EPSILON);
}

TEST(AngModPITest, HandlesZero) {
    EXPECT_NEAR(AngModPI(0.F), 0.0F, EPSILON);
}

TEST(AngModPITest, HandlesNegativeAngles) {
    EXPECT_NEAR(AngModPI(-M_PI_F), -M_PI_F, EPSILON);
    EXPECT_NEAR(AngModPI(-2.0F * M_PI_F), 0.0F, EPSILON);
    EXPECT_NEAR(AngModPI(-3.0F * M_PI_F), -M_PI_F, EPSILON);
}

TEST(AngModPITest, HandlesPositiveAngles) {
    EXPECT_NEAR(AngModPI(M_PI_F), M_PI_F, EPSILON);
    EXPECT_NEAR(AngModPI(2.0F * M_PI_F), 0.0F, EPSILON);
    EXPECT_NEAR(AngModPI(3.0F * M_PI_F), M_PI_F, EPSILON);
}