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


#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_GEO_ALIGNMENT_USE_CASE_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_GEO_ALIGNMENT_USE_CASE_HPP_

#include <mutex>
#include <optional>
#include <string>

#include "isaacbot_amr_vda5050_adapter/application/ports.hpp"
#include "isaacbot_amr_vda5050_adapter/domain/frame_aligner.hpp"
#include "isaacbot_amr_vda5050_adapter/domain/geo.hpp"
#include "isaacbot_amr_vda5050_adapter/domain/types.hpp"

namespace isaacbot_amr_vda5050_adapter::application
{

struct GeoAlignmentConfig
{
    domain::FrameAlignerConfig aligner;
    /// ENU yaw (rad, 0 = East, counter-clockwise) the isaacbot faces when parked at the shop; it
    /// seeds the yaw of a trip that starts there. Unset: the yaw waits for GNSS course or a fit.
    std::optional<double> parked_heading_enu;
    /// "At the shop": the first fix of a SLAM session lies within this distance of the datum.
    double parked_radius_m{30.0};
    /// Fixes with no accuracy figure count as this (1-sigma, metres).
    double default_sigma_m{3.0};
};

/**
 * @brief The GPS side of a delivery: the shared ENU frame, the latest fix, and its alignment to
 *        the live SLAM map. GPS is never a navigation source: it only places ENU orders into the
 *        SLAM map and reports where the isaacbot is on the fleet map.
 *
 * Thread-safe: fixes arrive on the executor thread, queries come from every handler.
 */
class GeoAlignmentUseCase : public FrameTransformPort
{
public:
    GeoAlignmentUseCase(domain::EnuDatum datum, GeoAlignmentConfig config);

    /** @brief A GNSS fix, with the isaacbot's navigation-frame pose at the time (if known). */
    void onFix(const domain::GpsFix & fix, const std::optional<domain::Pose2D> & map_pose);

    /**
     * @brief The isaacbot's measured ENU heading (isaacbot_amr_gps_heading's gps/heading_imu, from
     *        GNSS course). Replaces the parked-heading seed once per SLAM session: a measurement
     *        beats an assumption. Ignored once the samples fit the yaw themselves.
     */
    void onCourse(double heading_enu, const domain::Pose2D & map_pose);

    /** @brief The SLAM map restarted (new trip): the old alignment means nothing in it. */
    void reset();

    std::optional<domain::Transform2D> enuToNavigation() const override;

    /** @brief A navigation-frame pose in the ENU frame, when aligned. */
    std::optional<domain::Pose2D> toEnu(const domain::Pose2D & map_pose) const;

    std::optional<domain::GpsFix> latestFix() const;
    domain::AlignmentQuality quality() const;
    const domain::EnuDatum & datum() const { return datum_; }

    /** @brief 0..1 for VDA 5050 localizationScore: 1 = tight fit, 0 = no alignment. */
    double score() const;

private:
    domain::EnuDatum datum_;
    GeoAlignmentConfig config_;

    mutable std::mutex mutex_;
    domain::FrameAligner aligner_;
    std::optional<domain::GpsFix> latest_fix_;
    bool seeded_{false};
    bool seeded_by_course_{false};
    bool session_started_{false};
};

}  // namespace isaacbot_amr_vda5050_adapter::application

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_GEO_ALIGNMENT_USE_CASE_HPP_
