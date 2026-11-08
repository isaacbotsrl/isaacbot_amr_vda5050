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


// The same vectors as the delivery portal's tests/test_geo.py: both sides must agree on the frame.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "isaacbot_amr_vda5050_adapter/domain/geo.hpp"

using isaacbot_amr_vda5050_adapter::domain::EnuDatum;
using isaacbot_amr_vda5050_adapter::domain::EnuPoint;
using isaacbot_amr_vda5050_adapter::domain::GeoPoint;

namespace
{
const EnuDatum kDatum(GeoPoint{44.4268, 26.1025}, "enu_test");
}

TEST(Geo, OriginIsZero)
{
    const auto p = kDatum.toEnu(kDatum.origin());
    EXPECT_NEAR(p.x, 0.0, 1e-6);
    EXPECT_NEAR(p.y, 0.0, 1e-6);
}

TEST(Geo, NorthAndEastAxes)
{
    const auto north = kDatum.toEnu(GeoPoint{44.4278, 26.1025});
    const auto east = kDatum.toEnu(GeoPoint{44.4268, 26.1035});
    EXPECT_NEAR(north.x, 0.0, 0.01);
    EXPECT_NEAR(north.y, 111.12, 0.2);
    EXPECT_NEAR(east.y, 0.0, 0.01);
    EXPECT_NEAR(east.x, 79.65, 0.2);
}

TEST(Geo, RoundTripWithinOneCentimetre)
{
    for (const auto & p : {GeoPoint{44.4268, 26.1025}, GeoPoint{44.43, 26.11},
                           GeoPoint{44.40, 26.05}, GeoPoint{44.45, 26.15}})
    {
        const auto back = kDatum.toGeo(kDatum.toEnu(p));
        EXPECT_NEAR(back.lat, p.lat, 1e-7);
        EXPECT_NEAR(back.lon, p.lon, 1e-7);
    }
}

TEST(Geo, ValidatesTheDatum)
{
    EXPECT_FALSE((GeoPoint{91.0, 0.0}.valid()));
    EXPECT_FALSE((GeoPoint{NAN, 0.0}.valid()));
    EXPECT_THROW(EnuDatum(GeoPoint{0.0, 200.0}, "x"), std::invalid_argument);
}
