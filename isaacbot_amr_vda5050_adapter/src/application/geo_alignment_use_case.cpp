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


#include "isaacbot_amr_vda5050_adapter/application/geo_alignment_use_case.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace isaacbot_amr_vda5050_adapter::application
{

GeoAlignmentUseCase::GeoAlignmentUseCase(
    std::optional<domain::EnuDatum> datum, GeoAlignmentConfig config)
: datum_(std::move(datum)), config_(config), aligner_(config.aligner)
{
}

void GeoAlignmentUseCase::configure(
    domain::EnuDatum datum, std::optional<double> parked_heading_enu)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool same = datum_ && datum_->mapId() == datum.mapId() &&
        datum_->origin().lat == datum.origin().lat && datum_->origin().lon == datum.origin().lon &&
        config_.parked_heading_enu == parked_heading_enu;
    datum_ = std::move(datum);
    config_.parked_heading_enu = parked_heading_enu;
    if (!same) {
        resetLocked();
    }
}

std::optional<domain::EnuDatum> GeoAlignmentUseCase::datum() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return datum_;
}

std::string GeoAlignmentUseCase::mapId() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return datum_ ? datum_->mapId() : std::string();
}

void GeoAlignmentUseCase::onFix(
    const domain::GpsFix & fix, const std::optional<domain::Pose2D> & map_pose)
{
    if (!fix.point.valid()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_fix_ = fix;

    if (!map_pose || !datum_) {
        return;
    }

    const auto enu = datum_->toEnu(fix.point);
    const double sigma = fix.sigma_m > 0.0 ? fix.sigma_m : config_.default_sigma_m;

    // The first fix of a SLAM session: a trip that starts parked at the shop knows its heading.
    if (!session_started_) {
        session_started_ = true;
        if (!seeded_ && config_.parked_heading_enu &&
            std::hypot(enu.x, enu.y) <= config_.parked_radius_m)
        {
            aligner_.seed(*map_pose, *config_.parked_heading_enu, enu);
            seeded_ = true;
        }
    }

    aligner_.addFix(enu, sigma, *map_pose);
}

void GeoAlignmentUseCase::onCourse(double heading_enu, const domain::Pose2D & map_pose)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!datum_ || seeded_by_course_ || aligner_.quality().yaw_from_fit ||
        !std::isfinite(heading_enu))
    {
        return;
    }

    std::optional<domain::EnuPoint> position;
    if (latest_fix_) {
        position = datum_->toEnu(latest_fix_->point);
    }

    aligner_.seed(map_pose, heading_enu, position);
    seeded_ = true;
    seeded_by_course_ = true;
}

void GeoAlignmentUseCase::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    resetLocked();
}

void GeoAlignmentUseCase::resetLocked()
{
    aligner_.reset();
    latest_fix_.reset();
    seeded_ = false;
    seeded_by_course_ = false;
    session_started_ = false;
}

std::optional<domain::Transform2D> GeoAlignmentUseCase::enuToNavigation() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return aligner_.transform();
}

std::optional<domain::Pose2D> GeoAlignmentUseCase::toEnu(const domain::Pose2D & map_pose) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto transform = aligner_.transform();

    if (!transform) {
        return std::nullopt;
    }

    return transform->toEnu(map_pose);
}

std::optional<domain::GpsFix> GeoAlignmentUseCase::latestFix() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return latest_fix_;
}

domain::AlignmentQuality GeoAlignmentUseCase::quality() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    return aligner_.quality();
}

double GeoAlignmentUseCase::score() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!aligner_.transform()) {
        return 0.0;
    }

    const auto q = aligner_.quality();
    // 1 at a perfect fit, 0.5 at 3 m rms; a seed-only yaw caps it at 0.5.
    const double fit = 1.0 / (1.0 + std::pow(q.rms_m / 3.0, 2.0));

    return q.yaw_from_fit ? fit : std::min(fit, 0.5);
}

}  // namespace isaacbot_amr_vda5050_adapter::application
