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


#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "isaacbot_amr_vda5050_connector/vda_action.hpp"

#include "isaacbot_amr_vda5050_adapter/infrastructure/isaacbot_amr_link.hpp"

namespace isaacbot_amr_vda5050_adapter::plugins
{

/**
 * @brief startTrip: begin a delivery trip on a fresh live SLAM map, aligned to GPS.
 *
 * Parameters (from the delivery portal's settings, so the two never disagree):
 *   datum_lat, datum_lon  the pizzeria: origin of the shared ENU frame (both, or neither)
 *   enu_map_id            the ENU frame's mapId (the portal's orders use it)
 *   parked_heading_deg    compass bearing the isaacbot faces parked at the shop (optional)
 * Without them, the frame from the parameters (isaacbot.geo.*) is used, if any.
 *
 * The isaacbot navigates by lidar only and maps each trip afresh (no pre-built maps). This asks
 * isaacbot_amr_indoor_nav_manager for a new SLAM session, then waits until the isaacbot is mapping,
 * knows its pose and has a GPS alignment of the new map (seeded by isaacbot.geo.parked_heading_deg
 * when it is parked at the shop). FINISHED then: the delivery portal sends the order in the ENU
 * frame. FAILED after isaacbot.geo.start_trip_timeout, or when the manager refuses.
 */
class StartTrip : public adapter::VDAAction
{
public:
    void configure() override { link_ = infrastructure::IsaacbotLink::forNode(node_); }

    void reset(
        const isaacbot_amr_vda5050_msgs::msg::Action & action_msg,
        const std::shared_ptr<adapter::GoalHandleProcessVDAAction> goal_handle) override
    {
        // Before the goal is accepted, not on the action thread: a cancel that arrives before
        // that thread runs initialize() must not be lost.
        cancelled_ = false;
        VDAAction::reset(action_msg, goal_handle);
    }

    void initialize() override
    {
        if (cancelled_) {
            fail("Cancelled.");
            return;
        }
        if (link_->navigation().routeActive()) {
            fail("An order is still running; cancel it first.");
            return;
        }
        // Only now: a new frame resets the alignment, which a running route still needs.
        std::string why;
        if (!applyFrame(why)) {
            fail(why);
            return;
        }

        const auto sessions = link_->mappingSessions();
        const auto started = link_->startMapping();
        if (!started.ok) {
            fail("Could not start mapping: " + started.message);
            return;
        }
        if (cancelled_) {   // cancelled while start_mapping was answering
            fail("Cancelled.");
            return;
        }

        sessions_before_ = sessions;
        deadline_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(link_->config().start_trip_timeout));
        current_action_msg_.result_description = "Mapping afresh; waiting for the GPS alignment.";
        update_action_state(STATES::RUNNING);
    }

    void run() override
    {
        while (true) {
            if (link_->mappingSessions() > sessions_before_ && link_->readyForEnuOrders()) {
                current_action_msg_.result_description = "Mapping, aligned to GPS: ready.";
                update_action_state(STATES::FINISHED);
                return;
            }
            if (cancelled_) {
                fail("Cancelled.");
                return;
            }
            if (std::chrono::steady_clock::now() >= deadline_) {
                const auto gps = link_->gpsProblem();
                if (!gps.empty()) {
                    fail(gps);
                    return;
                }
                fail(link_->mappingSessions() > sessions_before_ ?
                    "No GPS alignment in time: no fix, or no heading (set "
                    "isaacbot.geo.parked_heading_deg)." :
                    "The new SLAM session did not start in time.");
                return;
            }
            std::this_thread::sleep_for(kPollPeriod);
        }
    }

    bool cancel() override
    {
        cancelled_ = true;
        return true;
    }

    void finish() override { goal_handle_->succeed(result_); }
    void fail() override { goal_handle_->abort(result_); }

private:
    static constexpr std::chrono::milliseconds kPollPeriod{200};

    /// The trip's ENU frame from the action parameters; false (and why) when there is none.
    bool applyFrame(std::string & why)
    {
        std::map<std::string, std::string> params;
        for (const auto & p : action_msg_.action_parameters) {
            params[p.key] = p.value;
        }
        const auto number = [&](const std::string & key) -> std::optional<double> {
            const auto it = params.find(key);
            if (it == params.end() || it->second.empty()) {
                return std::nullopt;
            }
            const double value = std::stod(it->second);   // throws on garbage: caught below
            if (!std::isfinite(value)) {
                throw std::invalid_argument(key);
            }
            return value;
        };

        try {
            const auto lat = number("datum_lat");
            const auto lon = number("datum_lon");
            const auto bearing = number("parked_heading_deg");
            if (lat.has_value() != lon.has_value()) {
                why = "startTrip needs both datum_lat and datum_lon, or neither.";
                return false;
            }
            if (!lat) {
                if (!link_->geo()->datum()) {
                    why = "No delivery ENU frame: startTrip carries none (datum_lat / datum_lon) "
                          "and isaacbot.geo.datum_lat/lon are unset.";
                    return false;
                }
                return true;   // the parameters' frame
            }
            std::string map_id = params.count("enu_map_id") ? params["enu_map_id"] : "";
            if (map_id.empty()) {
                map_id = link_->geo()->mapId().empty() ? link_->config().enu_map_id :
                                                        link_->geo()->mapId();
            }
            const auto heading = bearing ?
                std::optional<double>((90.0 - *bearing) * M_PI / 180.0) :
                link_->defaultParkedHeading();
            link_->configureDelivery(
                domain::EnuDatum(domain::GeoPoint{*lat, *lon}, map_id), heading);
            return true;
        } catch (const std::exception & e) {
            why = std::string("Bad startTrip parameter: ") + e.what();
            return false;
        }
    }

    void fail(const std::string & why)
    {
        current_action_msg_.result_description = why;
        update_action_state(STATES::FAILED);
    }

    std::shared_ptr<infrastructure::IsaacbotLink> link_;
    std::atomic<bool> cancelled_{false};
    std::uint64_t sessions_before_{0};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace isaacbot_amr_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::StartTrip, adapter::VDAAction)
