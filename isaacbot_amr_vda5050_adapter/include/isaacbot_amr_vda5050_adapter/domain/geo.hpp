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


#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_GEO_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_GEO_HPP_

#include <string>

namespace isaacbot_amr_vda5050_adapter::domain
{

/** @brief A WGS84 latitude / longitude in degrees. */
struct GeoPoint
{
    double lat{0.0};
    double lon{0.0};

    /** @brief Finite and within [-90, 90] / [-180, 180]. */
    bool valid() const;
};

/** @brief One GNSS fix. */
struct GpsFix
{
    GeoPoint point;
    double sigma_m{0.0};      ///< 1-sigma horizontal accuracy; 0 when unknown.
    std::string fix_type;     ///< GPS / SBAS / GBAS (RTK)
};

/** @brief A point of the shared East-North-Up frame: metres East (x) and North (y) of the datum. */
struct EnuPoint
{
    double x{0.0};
    double y{0.0};
};

/**
 * @brief The shared outdoor frame of the delivery fleet (VDA 5050 mapId `enu_<site>`).
 *
 * Its origin is a fixed datum, the pizzeria, on the WGS84 ellipsoid. The delivery portal
 * (isaacbot_amr_delivery_portal, domain/geo.py) does the same maths; test_geo.cpp checks the
 * same vectors as its tests/test_geo.py.
 */
class EnuDatum
{
public:
    /** @throws std::invalid_argument when @p origin is not a valid lat/lon. */
    EnuDatum(GeoPoint origin, std::string map_id);

    EnuPoint toEnu(const GeoPoint & point) const;
    GeoPoint toGeo(const EnuPoint & point) const;

    const GeoPoint & origin() const { return origin_; }
    const std::string & mapId() const { return map_id_; }

private:
    GeoPoint origin_;
    std::string map_id_;
};

}  // namespace isaacbot_amr_vda5050_adapter::domain

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_GEO_HPP_
