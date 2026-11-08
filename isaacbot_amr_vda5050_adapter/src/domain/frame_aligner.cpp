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


#include "isaacbot_amr_vda5050_adapter/domain/frame_aligner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace isaacbot_amr_vda5050_adapter::domain
{

double wrapAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2D Transform2D::toMap(const Pose2D & enu) const
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    return {c * enu.x - s * enu.y + tx, s * enu.x + c * enu.y + ty, wrapAngle(enu.theta + yaw)};
}

Pose2D Transform2D::toEnu(const Pose2D & map) const
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double x = map.x - tx;
    const double y = map.y - ty;

    return {c * x + s * y, -s * x + c * y, wrapAngle(map.theta - yaw)};
}

double displacement(const Transform2D & a, const Transform2D & b, const Pose2D & enu)
{
    const auto pa = a.toMap(enu);
    const auto pb = b.toMap(enu);

    return std::hypot(pa.x - pb.x, pa.y - pb.y);
}

FrameAligner::FrameAligner(FrameAlignerConfig config)
: config_(config)
{
}

void FrameAligner::reset()
{
    samples_.clear();
    seed_yaw_.reset();
    seed_transform_.reset();
    transform_.reset();
    quality_ = {};
}

void FrameAligner::seed(
    const Pose2D & map_pose, double heading_enu, const std::optional<EnuPoint> & position_enu)
{
    // ENU yaw = map yaw - transform yaw.
    seed_yaw_ = wrapAngle(map_pose.theta - heading_enu);

    if (position_enu) {
        Transform2D t{0.0, 0.0, *seed_yaw_};
        const auto rotated = t.toMap(Pose2D{position_enu->x, position_enu->y, 0.0});
        seed_transform_ = Transform2D{map_pose.x - rotated.x, map_pose.y - rotated.y, *seed_yaw_};
    }

    refit();
}

bool FrameAligner::addFix(const EnuPoint & gps, double sigma_m, const Pose2D & map_pose)
{
    if (!std::isfinite(gps.x) || !std::isfinite(gps.y) || !std::isfinite(sigma_m) ||
        sigma_m <= 0.0 || sigma_m > config_.max_sigma_m)
    {
        return false;
    }

    if (!samples_.empty()) {
        const auto & last = samples_.back().map;
        if (std::hypot(map_pose.x - last.x, map_pose.y - last.y) < config_.min_step_m) {
            return false;
        }
    }

    samples_.push_back(Sample{gps, map_pose, 1.0 / (sigma_m * sigma_m)});
    while (samples_.size() > config_.max_samples) {
        samples_.pop_front();
    }

    refit();
    return true;
}

void FrameAligner::refit()
{
    std::vector<Sample> used(samples_.begin(), samples_.end());

    if (used.empty()) {
        transform_ = seed_transform_;
        quality_ = {};
        return;
    }

    for (int pass = 0; pass < 2; ++pass) {
        double w_sum = 0.0;
        double ex = 0.0, ey = 0.0, mx = 0.0, my = 0.0;
        for (const auto & s : used) {
            w_sum += s.weight;
            ex += s.weight * s.gps.x;
            ey += s.weight * s.gps.y;
            mx += s.weight * s.map.x;
            my += s.weight * s.map.y;
        }
        ex /= w_sum;
        ey /= w_sum;
        mx /= w_sum;
        my /= w_sum;

        double cross = 0.0, dot = 0.0, spread = 0.0;
        for (const auto & s : used) {
            const double ax = s.gps.x - ex, ay = s.gps.y - ey;
            const double bx = s.map.x - mx, by = s.map.y - my;
            cross += s.weight * (ax * by - ay * bx);
            dot += s.weight * (ax * bx + ay * by);
            // Spread in the SLAM frame: GPS noise alone must not make a parked robot "spread".
            spread = std::max(spread, std::hypot(bx, by));
        }
        spread *= 2.0;   // radius around the centroid -> span of the path driven

        double yaw = 0.0;
        bool from_fit = spread >= config_.min_spread_m;
        if (from_fit) {
            yaw = std::atan2(cross, dot);
        } else if (seed_yaw_) {
            yaw = *seed_yaw_;
        } else {
            transform_.reset();
            quality_ = {used.size(), 0.0, spread, false};
            return;
        }

        const double c = std::cos(yaw), s = std::sin(yaw);
        Transform2D t{mx - (c * ex - s * ey), my - (s * ex + c * ey), yaw};

        double sq = 0.0;
        std::vector<double> residuals;
        residuals.reserve(used.size());
        for (const auto & sample : used) {
            const auto p = t.toMap(Pose2D{sample.gps.x, sample.gps.y, 0.0});
            const double r = std::hypot(p.x - sample.map.x, p.y - sample.map.y);
            residuals.push_back(r);
            sq += r * r;
        }
        const double rms = std::sqrt(sq / static_cast<double>(used.size()));

        transform_ = t;
        quality_ = {used.size(), rms, spread, from_fit};

        if (pass == 0) {
            const double gate = std::max(config_.outlier_gate_m, 3.0 * rms);
            std::vector<Sample> kept;
            for (std::size_t i = 0; i < used.size(); ++i) {
                if (residuals[i] <= gate) {
                    kept.push_back(used[i]);
                }
            }
            if (kept.size() == used.size() || kept.size() < 3) {
                return;
            }
            used = std::move(kept);
        }
    }
}

}  // namespace isaacbot_amr_vda5050_adapter::domain
