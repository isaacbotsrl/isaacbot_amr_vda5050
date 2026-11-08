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


#include "isaacbot_amr_vda5050_adapter/domain/geo.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace isaacbot_amr_vda5050_adapter::domain
{

namespace
{

// WGS84
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDegToRad = M_PI / 180.0;

struct Ecef
{
    double x;
    double y;
    double z;
};

Ecef toEcef(double lat_deg, double lon_deg)
{
    const double lat = lat_deg * kDegToRad;
    const double lon = lon_deg * kDegToRad;
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));

    return {n * std::cos(lat) * std::cos(lon), n * std::cos(lat) * std::sin(lon),
        n * (1.0 - kE2) * std::sin(lat)};
}

GeoPoint toGeodetic(const Ecef & e)
{
    const double lon = std::atan2(e.y, e.x);
    const double p = std::hypot(e.x, e.y);
    double lat = std::atan2(e.z, p * (1.0 - kE2));

    for (int i = 0; i < 6; ++i) {
        const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
        const double h = p / std::cos(lat) - n;
        lat = std::atan2(e.z, p * (1.0 - kE2 * n / (n + h)));
    }

    return {lat / kDegToRad, lon / kDegToRad};
}

}  // namespace

bool GeoPoint::valid() const
{
    return std::isfinite(lat) && std::isfinite(lon) && lat >= -90.0 && lat <= 90.0 &&
           lon >= -180.0 && lon <= 180.0;
}

EnuDatum::EnuDatum(GeoPoint origin, std::string map_id)
: origin_(origin), map_id_(std::move(map_id))
{
    if (!origin_.valid()) {
        throw std::invalid_argument("The ENU datum is not a valid latitude/longitude.");
    }
}

EnuPoint EnuDatum::toEnu(const GeoPoint & point) const
{
    const Ecef o = toEcef(origin_.lat, origin_.lon);
    const Ecef p = toEcef(point.lat, point.lon);
    const double dx = p.x - o.x;
    const double dy = p.y - o.y;
    const double dz = p.z - o.z;
    const double lat = origin_.lat * kDegToRad;
    const double lon = origin_.lon * kDegToRad;

    return {-std::sin(lon) * dx + std::cos(lon) * dy,
        -std::sin(lat) * std::cos(lon) * dx - std::sin(lat) * std::sin(lon) * dy +
        std::cos(lat) * dz};
}

GeoPoint EnuDatum::toGeo(const EnuPoint & point) const
{
    const Ecef o = toEcef(origin_.lat, origin_.lon);
    const double lat = origin_.lat * kDegToRad;
    const double lon = origin_.lon * kDegToRad;

    // On the tangent plane (up = 0), then back onto the ellipsoid.
    return toGeodetic(Ecef{
        o.x - std::sin(lon) * point.x - std::sin(lat) * std::cos(lon) * point.y,
        o.y + std::cos(lon) * point.x - std::sin(lat) * std::sin(lon) * point.y,
        o.z + std::cos(lat) * point.y});
}

}  // namespace isaacbot_amr_vda5050_adapter::domain
