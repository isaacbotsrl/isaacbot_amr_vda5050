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


#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_FRAME_ALIGNER_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_FRAME_ALIGNER_HPP_

#include <cstddef>
#include <deque>
#include <optional>

#include "isaacbot_amr_vda5050_adapter/domain/geo.hpp"
#include "isaacbot_amr_vda5050_adapter/domain/types.hpp"

namespace isaacbot_amr_vda5050_adapter::domain
{

/** @brief A planar rigid transform: map = R(yaw) * enu + t. */
struct Transform2D
{
    double tx{0.0};
    double ty{0.0};
    double yaw{0.0};

    /** @brief A pose of the ENU frame (theta = ENU yaw) in the navigation (SLAM) frame. */
    Pose2D toMap(const Pose2D & enu) const;
    /** @brief A navigation-frame pose in the ENU frame. */
    Pose2D toEnu(const Pose2D & map) const;
};

/** @brief How well the alignment is known. */
struct AlignmentQuality
{
    std::size_t samples{0};
    double rms_m{0.0};          ///< Residual of the GPS fixes against the fit.
    double spread_m{0.0};       ///< Span of the path sampled (SLAM frame): below min_spread, yaw is the seed's.
    bool yaw_from_fit{false};   ///< false: the yaw is the seed's (heading when parked).
};

struct FrameAlignerConfig
{
    double min_step_m{2.0};      ///< A new sample once the robot moved this far (navigation frame).
    std::size_t max_samples{400};  ///< The most recent ones are kept: SLAM drifts, old pairs go stale.
    double min_spread_m{15.0};   ///< Fit the yaw only once the samples span this much.
    double max_sigma_m{8.0};     ///< Ignore fixes worse than this (1-sigma horizontal).
    double outlier_gate_m{4.0};  ///< Drop pairs whose residual exceeds max(this, 3 x rms), once.
};

/**
 * @brief Keeps the transform from the shared ENU frame (GPS) to the robot's live SLAM map.
 *
 * The robot navigates by lidar only; GPS never feeds Nav 2. This only answers "where is that ENU
 * waypoint in my SLAM map", so delivery orders can be driven in the SLAM frame. It is a weighted
 * 2D rigid least-squares fit (Umeyama, no scale) of (GPS fix in ENU, SLAM pose) pairs taken as the
 * robot drives. Until the samples spread enough to give the yaw, the yaw comes from the seed: the
 * robot's known heading when the trip starts (parked at the shop).
 *
 * A fresh SLAM session (new trip) is a new map frame: reset() then.
 */
class FrameAligner
{
public:
    explicit FrameAligner(FrameAlignerConfig config = {});

    /** @brief Forget everything (a new SLAM session). */
    void reset();

    /**
     * @brief The robot at @p map_pose has ENU yaw @p heading_enu: sets the yaw used until the
     *        samples can fit one. With no samples yet, @p position_enu gives the translation.
     */
    void seed(const Pose2D & map_pose, double heading_enu, const std::optional<EnuPoint> & position_enu);

    /**
     * @brief One GPS fix (in ENU, 1-sigma @p sigma_m) taken with the robot at @p map_pose.
     * @return true when it was kept as a sample.
     */
    bool addFix(const EnuPoint & gps, double sigma_m, const Pose2D & map_pose);

    /** @brief ENU -> map, or nothing until there is a yaw (seed or fit) and a position. */
    std::optional<Transform2D> transform() const { return transform_; }
    AlignmentQuality quality() const { return quality_; }

private:
    struct Sample
    {
        EnuPoint gps;
        Pose2D map;
        double weight;
    };

    void refit();

    FrameAlignerConfig config_;
    std::deque<Sample> samples_;
    std::optional<double> seed_yaw_;
    std::optional<Transform2D> seed_transform_;
    std::optional<Transform2D> transform_;
    AlignmentQuality quality_;
};

/** @brief How far a pose moves between two transforms (position only). */
double displacement(const Transform2D & a, const Transform2D & b, const Pose2D & enu);

/** @brief The angle wrapped into (-pi, pi]. */
double wrapAngle(double angle);

}  // namespace isaacbot_amr_vda5050_adapter::domain

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_DOMAIN_FRAME_ALIGNER_HPP_
