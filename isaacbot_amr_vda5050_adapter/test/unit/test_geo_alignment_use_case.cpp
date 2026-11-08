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

#include "isaacbot_amr_vda5050_adapter/application/geo_alignment_use_case.hpp"

using isaacbot_amr_vda5050_adapter::application::GeoAlignmentConfig;
using isaacbot_amr_vda5050_adapter::application::GeoAlignmentUseCase;
using isaacbot_amr_vda5050_adapter::domain::EnuDatum;
using isaacbot_amr_vda5050_adapter::domain::EnuPoint;
using isaacbot_amr_vda5050_adapter::domain::GeoPoint;
using isaacbot_amr_vda5050_adapter::domain::GpsFix;
using isaacbot_amr_vda5050_adapter::domain::Pose2D;

namespace
{

const EnuDatum kDatum(GeoPoint{44.4268, 26.1025}, "enu_test");

GpsFix fixAt(double east, double north, double sigma = 1.0)
{
    return GpsFix{kDatum.toGeo(EnuPoint{east, north}), sigma, "GPS"};
}

GeoAlignmentConfig parkedFacingNorth()
{
    GeoAlignmentConfig config;
    config.parked_heading_enu = M_PI_2;
    return config;
}

}  // namespace

TEST(GeoAlignmentUseCase, ParkedAtTheShopIsAlignedFromTheFirstFix)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    EXPECT_FALSE(geo.enuToNavigation());

    geo.onFix(fixAt(1.0, 0.0), Pose2D{0, 0, 0});

    const auto t = geo.enuToNavigation();
    ASSERT_TRUE(t);
    // 20 m North of the shop is ~20 m ahead of where the SLAM map started.
    const auto ahead = t->toMap(Pose2D{1.0, 20.0, 0});
    EXPECT_NEAR(ahead.x, 20.0, 1e-6);
    EXPECT_NEAR(ahead.y, 0.0, 1e-6);
    EXPECT_GT(geo.score(), 0.0);
    EXPECT_LE(geo.score(), 0.5);   // seeded yaw only
    ASSERT_TRUE(geo.latestFix());
    ASSERT_TRUE(geo.toEnu(Pose2D{0, 0, 0}));
}

TEST(GeoAlignmentUseCase, NoParkedSeedAwayFromTheShop)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    geo.onFix(fixAt(500.0, 0.0), Pose2D{0, 0, 0});
    EXPECT_FALSE(geo.enuToNavigation());

    // A measured GNSS course seeds it instead.
    geo.onCourse(0.0, Pose2D{0, 0, 0});
    EXPECT_TRUE(geo.enuToNavigation());
}

TEST(GeoAlignmentUseCase, CourseReplacesTheParkedSeedOnce)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    geo.onFix(fixAt(0.0, 0.0), Pose2D{0, 0, 0});
    geo.onCourse(0.0, Pose2D{0, 0, 0});   // actually facing East
    EXPECT_NEAR(geo.enuToNavigation()->yaw, 0.0, 1e-9);
    geo.onCourse(1.0, Pose2D{0, 0, 0});   // later courses do not re-seed
    EXPECT_NEAR(geo.enuToNavigation()->yaw, 0.0, 1e-9);
}

TEST(GeoAlignmentUseCase, ResetStartsANewSession)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    geo.onFix(fixAt(0.0, 0.0), Pose2D{0, 0, 0});
    ASSERT_TRUE(geo.enuToNavigation());

    geo.reset();
    EXPECT_FALSE(geo.enuToNavigation());
    geo.onFix(fixAt(2.0, 0.0), Pose2D{5, 5, 0});   // first fix of the new map: parked seed again
    EXPECT_TRUE(geo.enuToNavigation());
}

TEST(GeoAlignmentUseCase, IgnoresInvalidFixesAndKeepsFixesWithoutPose)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    geo.onFix(GpsFix{GeoPoint{NAN, 0.0}, 1.0, "GPS"}, Pose2D{});
    EXPECT_FALSE(geo.latestFix());
    geo.onFix(fixAt(0.0, 0.0), std::nullopt);   // no SLAM pose yet: shown, not aligned
    EXPECT_TRUE(geo.latestFix());
    EXPECT_FALSE(geo.enuToNavigation());
}

TEST(GeoAlignmentUseCase, ResetForgetsTheLastFix)
{
    GeoAlignmentUseCase geo(kDatum, parkedFacingNorth());
    geo.onFix(fixAt(0.0, 0.0), Pose2D{0, 0, 0});
    geo.reset();
    EXPECT_FALSE(geo.latestFix());
}
