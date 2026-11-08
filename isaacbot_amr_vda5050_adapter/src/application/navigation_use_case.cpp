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

#include "isaacbot_amr_vda5050_adapter/application/navigation_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace isaacbot_amr_vda5050_adapter::application
{

using domain::Dispatch;
using domain::RouteOutcome;

NavigationUseCase::NavigationUseCase(
    std::shared_ptr<MissionPort> missions, std::shared_ptr<PosePort> pose,
    std::string mission_prefix, domain::OrientationMode orientation_mode,
    std::shared_ptr<FrameTransformPort> frames, RealignConfig realign)
: missions_(std::move(missions)),
  pose_(std::move(pose)),
  frames_(std::move(frames)),
  realign_(realign),
  tracker_(std::move(mission_prefix), orientation_mode)
{
    if (missions_ == nullptr || pose_ == nullptr) {
        throw std::invalid_argument("NavigationUseCase requires a MissionPort and a PosePort");
    }
}

CommandResult NavigationUseCase::start(
    std::vector<domain::RouteNode> nodes, domain::RouteFrame frame)
{
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    const auto pose = pose_->currentPose();

    std::optional<domain::Transform2D> to_navigation;
    if (frame == domain::RouteFrame::kEnu) {
        if (frames_ != nullptr) {
            to_navigation = frames_->enuToNavigation();
        }
        if (!to_navigation) {
            return {false,
                "The order is in the shared ENU frame, but the isaacbot has no GPS alignment to "
                "its map yet (no GPS fix, or no heading).", false};
        }
    }

    std::optional<Dispatch> dispatch;
    bool ours_was_running = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        updates_.clear();
        ours_was_running = tracker_.dispatched();

        try {
            dispatch = tracker_.start(std::move(nodes), pose, to_navigation);
        } catch (const std::exception & e) {
            return {false, e.what(), false};
        }
    }

    return send(dispatch, ours_was_running);
}

CommandResult NavigationUseCase::extend(std::vector<domain::RouteNode> nodes)
{
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    const auto pose = pose_->currentPose();

    std::optional<Dispatch> dispatch;
    bool ours_was_running = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ours_was_running = tracker_.dispatched();

        try {
            dispatch = tracker_.extend(nodes, pose);
        } catch (const std::exception & e) {
            return {false, e.what(), false};
        }
    }

    return send(dispatch, ours_was_running);
}

CommandResult NavigationUseCase::realign()
{
    if (frames_ == nullptr) {
        return {true, "No ENU frame.", false};
    }

    const auto to_navigation = frames_->enuToNavigation();

    if (!to_navigation) {
        // Only a new SLAM session drops the alignment: the route's goals belong to a dead frame.
        if (!routeInEnu()) {
            return {true, "No GPS alignment.", false};
        }
        const auto cancelled = cancel();
        return {false,
            "The GPS alignment was lost (a new map started): the route was stopped. " +
            cancelled.message, false};
    }

    std::lock_guard<std::mutex> command_lock(command_mutex_);

    const auto pose = pose_->currentPose();
    std::optional<Dispatch> dispatch;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        dispatch = tracker_.realign(
            *to_navigation, pose, realign_.min_shift_m, realign_.min_yaw_rad);
    }

    if (!dispatch) {
        return {true, "Alignment unchanged.", false};
    }

    auto result = send(dispatch, true);
    if (result.ok) {
        result.message = "Re-projected the remaining nodes onto the map: " + result.message;
    }

    return result;
}

bool NavigationUseCase::routeInEnu() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return tracker_.active() && tracker_.transform().has_value();
}

CommandResult NavigationUseCase::cancel()
{
    std::lock_guard<std::mutex> command_lock(command_mutex_);

    bool had_mission = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        had_mission = tracker_.dispatched();
        // Forget the route first, so the CANCELLED the manager publishes is not read as a
        // failure of the isaacbot.
        tracker_.clear();
        updates_.clear();
        // Master control has dealt with the order; a refusal of it is no longer news.
        last_refusal_.reset();
    }

    if (!had_mission) {
        return {true, "No mission to cancel.", false};
    }

    return missions_->cancel();
}

CommandResult NavigationUseCase::pause()
{
    std::lock_guard<std::mutex> command_lock(command_mutex_);

    bool had_mission = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        had_mission = tracker_.pause();
    }

    if (!had_mission) {
        return {true, "Paused.", false};
    }

    const auto result = missions_->cancel();

    if (!result.ok) {
        return {false, "Could not stop the running mission: " + result.message, false};
    }

    return {true, "Paused; the running mission was stopped.", false};
}

CommandResult NavigationUseCase::resume()
{
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    const auto pose = pose_->currentPose();

    std::optional<Dispatch> dispatch;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        dispatch = tracker_.resume(pose);
    }

    if (!dispatch) {
        return {true, "Resumed; no route to continue.", false};
    }

    // Paused means the previous mission was already cancelled.
    return send(dispatch, false);
}

void NavigationUseCase::onMissionProgress(const domain::MissionProgress & progress)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto update = tracker_.onProgress(progress);

    if (update.last_reached || update.outcome != RouteOutcome::kInProgress) {
        updates_.push_back(std::move(update));
    }
}

std::vector<domain::RouteUpdate> NavigationUseCase::takeUpdates()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<domain::RouteUpdate> updates;
    updates.swap(updates_);

    return updates;
}

bool NavigationUseCase::routeActive() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return tracker_.active();
}

bool NavigationUseCase::paused() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return tracker_.paused() || tracker_.heldByIsaacbot();
}

std::optional<std::string> NavigationUseCase::lastRefusal() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    return last_refusal_;
}

CommandResult NavigationUseCase::send(
    const std::optional<Dispatch> & dispatch, bool ours_was_running)
{
    if (!dispatch) {
        return {true, "Paused; the route continues on stopPause.", false};
    }

    const auto result = missions_->dispatch(dispatch->mission_id, dispatch->waypoints);

    if (result.ok) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_refusal_.reset();

        return result;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_refusal_ = result.message;
        tracker_.clear();
        updates_.clear();
    }

    // The route is gone. A refused set_mission leaves the manager's current mission alone, which
    // is ours to stop only if this route had dispatched it - it may equally be an operator's
    // GoTo. A call that timed out may have started the new mission, so that one is stopped too.
    if (ours_was_running || result.outcome_unknown) {
        missions_->cancel();
    }

    return result;
}

}  // namespace isaacbot_amr_vda5050_adapter::application
