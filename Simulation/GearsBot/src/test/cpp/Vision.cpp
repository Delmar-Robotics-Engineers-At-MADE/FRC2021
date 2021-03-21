#include <gtest/gtest.h>
#include "Vision.h"

class VisionTest : public testing::Test {
protected:
};

TEST_F(VisionTest, BallSort) {
    VisionSubsystem *vs = new VisionSubsystem();
    std::string ballAngles = vs->sortFakeBallAngles();
    EXPECT_EQ(ballAngles, "37.964 37.964 37.964 ");
    delete(vs);
}
