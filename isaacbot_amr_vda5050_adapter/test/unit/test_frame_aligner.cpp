// Copyright 2026 IsaacBot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "isaacbot_amr_vda5050_adapter/domain/frame_aligner.hpp"

using isaacbot_amr_vda5050_adapter::domain::EnuPoint;
using isaacbot_amr_vda5050_adapter::domain::FrameAligner;
using isaacbot_amr_vda5050_adapter::domain::FrameAlignerConfig;
using isaacbot_amr_vda5050_adapter::domain::Pose2D;
using isaacbot_amr_vda5050_adapter::domain::Transform2D;
using isaacbot_amr_vda5050_adapter::domain::displacement;
using isaacbot_amr_vda5050_adapter::domain::wrapAngle;

namespace
{

// The SLAM map started 40 m East, 10 m North of the shop, rotated 30 degrees.
const Transform2D kTruth{-25.0, 18.0, 0.5236};

/// Drive an L (east 60 m, then north 40 m) and feed noisy fixes every metre.
void drive(FrameAligner & aligner, const Transform2D & truth, double noise, unsigned seed = 7)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, noise);
    for (int i = 0; i <= 100; ++i) {
        const Pose2D enu = i <= 60 ? Pose2D{i * 1.0, 0.0, 0.0} : Pose2D{60.0, (i - 60) * 1.0, M_PI_2};
        const auto map = truth.toMap(enu);
        aligner.addFix(EnuPoint{enu.x + gauss(rng), enu.y + gauss(rng)}, noise > 0 ? noise : 0.5, map);
    }
}

}  // namespace

TEST(Transform2D, RoundTrip)
{
    const Pose2D p{3.0, -4.0, 1.0};
    const auto back = kTruth.toEnu(kTruth.toMap(p));
    EXPECT_NEAR(back.x, p.x, 1e-9);
    EXPECT_NEAR(back.y, p.y, 1e-9);
    EXPECT_NEAR(back.theta, p.theta, 1e-9);
    EXPECT_NEAR(kTruth.toMap(Pose2D{0, 0, 0}).theta, 0.5236, 1e-9);
}

TEST(FrameAligner, NothingWithoutHeadingOrSpread)
{
    FrameAligner aligner;
    aligner.addFix(EnuPoint{0, 0}, 1.0, Pose2D{0, 0, 0});
    EXPECT_FALSE(aligner.transform());
    EXPECT_EQ(aligner.quality().samples, 1U);
}

TEST(FrameAligner, RecoversAKnownTransformFromNoisyFixes)
{
    FrameAligner aligner;
    drive(aligner, kTruth, 2.0);

    const auto t = aligner.transform();
    ASSERT_TRUE(t);
    EXPECT_TRUE(aligner.quality().yaw_from_fit);
    EXPECT_NEAR(wrapAngle(t->yaw - kTruth.yaw), 0.0, 0.05);
    // Where it matters: the destination 100 m out lands within a metre or two.
    EXPECT_LT(displacement(*t, kTruth, Pose2D{100.0, 40.0, 0.0}), 2.5);
    EXPECT_GT(aligner.quality().rms_m, 1.0);
    EXPECT_LT(aligner.quality().rms_m, 4.0);
}

TEST(FrameAligner, SeedGivesTheYawUntilTheSamplesSpread)
{
    FrameAligner aligner;
    // Parked at the shop (ENU origin), facing North (ENU yaw pi/2); SLAM starts at its origin.
    aligner.seed(Pose2D{0, 0, 0}, M_PI_2, EnuPoint{0, 0});
    auto t = aligner.transform();
    ASSERT_TRUE(t);
    EXPECT_NEAR(t->yaw, -M_PI_2, 1e-9);
    // 10 m North in ENU is 10 m straight ahead (+x) in the SLAM map.
    const auto ahead = t->toMap(Pose2D{0, 10, M_PI_2});
    EXPECT_NEAR(ahead.x, 10.0, 1e-9);
    EXPECT_NEAR(ahead.y, 0.0, 1e-9);
    EXPECT_NEAR(ahead.theta, 0.0, 1e-9);

    aligner.addFix(EnuPoint{0.5, 0.2}, 2.0, Pose2D{0, 0, 0});   // still parked: seed yaw
    EXPECT_FALSE(aligner.quality().yaw_from_fit);
    EXPECT_NEAR(aligner.transform()->yaw, -M_PI_2, 1e-9);
}

TEST(FrameAligner, DropsOutliersAndBadFixes)
{
    FrameAligner aligner;
    drive(aligner, kTruth, 0.0);
    EXPECT_FALSE(aligner.addFix(EnuPoint{1e3, 1e3}, 20.0, kTruth.toMap(Pose2D{200, 0, 0})));
    // A multipath jump inside the sigma limit is accepted as a sample, then rejected by the fit.
    ASSERT_TRUE(aligner.addFix(EnuPoint{90.0, 40.0}, 3.0, kTruth.toMap(Pose2D{60.0, 50.0, 0})));
    const auto t = aligner.transform();
    ASSERT_TRUE(t);
    EXPECT_LT(displacement(*t, kTruth, Pose2D{60, 50, 0}), 0.3);
}

TEST(FrameAligner, FollowsSlamDriftThroughTheSampleWindow)
{
    FrameAlignerConfig config;
    config.max_samples = 30;
    FrameAligner aligner(config);
    drive(aligner, kTruth, 0.0);
    // The SLAM map has drifted 3 m: the recent window follows it.
    Transform2D drifted = kTruth;
    drifted.tx += 3.0;
    // 100 m at a sample per 2 m turns the 30-sample window over completely.
    for (int i = 0; i <= 100; ++i) {
        const Pose2D enu{60.0 - i, 40.0, M_PI};
        aligner.addFix(EnuPoint{enu.x, enu.y}, 0.5, drifted.toMap(enu));
    }
    EXPECT_LT(displacement(*aligner.transform(), drifted, Pose2D{0, 40, 0}), 0.2);
}

TEST(FrameAligner, ResetForgetsEverything)
{
    FrameAligner aligner;
    drive(aligner, kTruth, 0.0);
    aligner.reset();
    EXPECT_FALSE(aligner.transform());
    EXPECT_EQ(aligner.quality().samples, 0U);
}
