#include <gtest/gtest.h>
#include "nav_bridge/cyvet/motion_math.hpp"

TEST(CyvetVelocity, IntersectionAndInvalidInputs) {
    using nav_bridge::cyvet::limitAxis;
    EXPECT_DOUBLE_EQ(limitAxis(-1,0.4,{-0.2,1.5}),-0.2);
    EXPECT_DOUBLE_EQ(limitAxis(1,0.4,{-0.2,1.5}),0.4);
    EXPECT_THROW(limitAxis(NAN,0.4,{-1,1}),std::invalid_argument);
    EXPECT_THROW(limitAxis(0,0.4,{1,2}),std::invalid_argument);
}
TEST(CyvetOdometry, ReentryRetainsPoseAndRotatesDisplacement) {
    nav_bridge::cyvet::ContinuousOdometry odom;
    const auto last=odom.update(1,{3,4,1.5707963267948966});
    const auto first=odom.update(2,{0,0,0});
    EXPECT_NEAR(last.x,first.x,1e-9); EXPECT_NEAR(last.y,first.y,1e-9);
    const auto next=odom.update(2,{1,0,0});
    EXPECT_NEAR(next.x,3,1e-9); EXPECT_NEAR(next.y,5,1e-9);
    odom.newSession();
    const auto resumed=odom.update(2,{0,0,0});
    EXPECT_NEAR(resumed.x,next.x,1e-9); EXPECT_NEAR(resumed.y,next.y,1e-9);
}

TEST(CyvetOdometry, FirstSessionDoesNotSuppressSecondDisplacement) {
    nav_bridge::cyvet::ContinuousOdometry odom;
    odom.newSession();
    odom.update(1,{0,0,0});
    EXPECT_DOUBLE_EQ(odom.update(1,{1,0,0}).x,1);
}
