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

#include "isaacbot_amr_vda5050_adapter/infrastructure/isaacbot_amr_link.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.hpp"

#include "isaacbot_amr_vda5050_adapter/domain/aux_outputs.hpp"
#include "isaacbot_amr_vda5050_adapter/infrastructure/aux_output_client.hpp"
#include "isaacbot_amr_vda5050_adapter/infrastructure/drive_mode_client.hpp"
#include "isaacbot_amr_vda5050_adapter/infrastructure/follow_me_client.hpp"
#include "isaacbot_amr_vda5050_adapter/infrastructure/mission_manager_client.hpp"

namespace isaacbot_amr_vda5050_adapter::infrastructure
{

namespace
{

std::string stripSlashes(const std::string & name)
{
    const auto first = name.find_first_not_of('/');
    if (first == std::string::npos) {
        return {};
    }

    const auto last = name.find_last_not_of('/');
    return name.substr(first, last - first + 1);
}

// Topic and service names are relative to the isaacbot namespace, not to the adapter node's own
// (the connector runs under <namespace>/vda5050). Absolute names are used as given.
std::string resolveName(const std::string & isaacbot_amr_namespace, const std::string & name)
{
    if (!name.empty() && name.front() == '/') {
        return name;
    }

    const auto ns = stripSlashes(isaacbot_amr_namespace);
    return ns.empty() ? "/" + name : "/" + ns + "/" + name;
}

// Frames follow the isaacbot's tf_prefix convention (<namespace>/map); a frame that already names a
// prefix is used as given.
std::string resolveFrame(const std::string & isaacbot_amr_namespace, const std::string & frame)
{
    const auto stripped = stripSlashes(frame);

    if (stripped.find('/') != std::string::npos) {
        return stripped;
    }

    const auto ns = stripSlashes(isaacbot_amr_namespace);
    return ns.empty() ? stripped : ns + "/" + stripped;
}

domain::OrientationMode parseOrientationMode(const std::string & mode)
{
    if (mode == "node") {
        return domain::OrientationMode::kNode;
    }

    if (mode == "path") {
        return domain::OrientationMode::kPath;
    }

    throw std::invalid_argument(
        "isaacbot.orientation_mode must be 'path' or 'node', got '" + mode + "'");
}

domain::MissionPhase phaseFor(std::uint8_t state)
{
    using isaacbot_amr_msgs::msg::MissionState;

    switch (state) {
        case MissionState::RUNNING:
            return domain::MissionPhase::kRunning;
        case MissionState::HELD:
            return domain::MissionPhase::kHeld;
        case MissionState::SUCCEEDED:
            return domain::MissionPhase::kSucceeded;
        case MissionState::FAILED:
            return domain::MissionPhase::kFailed;
        case MissionState::CANCELLED:
            return domain::MissionPhase::kCancelled;
        default:
            return domain::MissionPhase::kIdle;
    }
}

domain::DriveMode driveModeFor(std::uint8_t mode)
{
    using isaacbot_amr_msgs::msg::DriveMode;

    switch (mode) {
        case DriveMode::MANUAL:
            return domain::DriveMode::kManual;
        case DriveMode::ASSISTED:
            return domain::DriveMode::kAssisted;
        case DriveMode::AUTOMATIC:
            return domain::DriveMode::kAutomatic;
        default:
            return domain::DriveMode::kUnknown;
    }
}

domain::Guard guardFor(std::uint8_t guard)
{
    using isaacbot_amr_msgs::msg::DriveMode;

    switch (guard) {
        case DriveMode::GUARD_BYPASSED:
            return domain::Guard::kBypassed;
        case DriveMode::GUARD_CLEAR:
            return domain::Guard::kClear;
        case DriveMode::GUARD_SLOWING:
            return domain::Guard::kSlowing;
        case DriveMode::GUARD_STOPPED:
            return domain::Guard::kStopped;
        case DriveMode::GUARD_NO_DATA:
            return domain::Guard::kNoData;
        default:
            return domain::Guard::kUnknown;
    }
}

domain::BatteryReading batteryReadingFor(const sensor_msgs::msg::BatteryState & msg)
{
    using sensor_msgs::msg::BatteryState;

    domain::BatteryReading reading;

    // Same rule as isaacbot_amr_mission_manager and isaacbot_amr_safety: an UNKNOWN-status report carries no
    // real charge.
    if (msg.power_supply_status != BatteryState::POWER_SUPPLY_STATUS_UNKNOWN &&
        std::isfinite(msg.percentage))
    {
        reading.fraction = msg.percentage;
    }

    if (std::isfinite(msg.voltage) && msg.voltage > 0.0F) {
        reading.voltage = msg.voltage;
    }

    reading.charging = msg.power_supply_status == BatteryState::POWER_SUPPLY_STATUS_CHARGING;

    return reading;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

constexpr double kDegToRad = M_PI / 180.0;
// After a new SLAM session starts, GPS fixes wait this long before pairing with TF poses.
constexpr double kSessionSettle = 2.0;

std::optional<domain::GpsFix> gpsFixFor(const sensor_msgs::msg::NavSatFix & msg)
{
    using sensor_msgs::msg::NavSatFix;
    using sensor_msgs::msg::NavSatStatus;

    if (msg.status.status < NavSatStatus::STATUS_FIX) {
        return std::nullopt;
    }

    domain::GpsFix fix;
    fix.point = domain::GeoPoint{msg.latitude, msg.longitude};
    if (!fix.point.valid()) {
        return std::nullopt;
    }

    if (msg.position_covariance_type != NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        const double variance = std::max(msg.position_covariance[0], msg.position_covariance[4]);
        if (std::isfinite(variance) && variance > 0.0) {
            fix.sigma_m = std::sqrt(variance);
        }
    }

    switch (msg.status.status) {
        case NavSatStatus::STATUS_GBAS_FIX:
            fix.fix_type = "GBAS";
            break;
        case NavSatStatus::STATUS_SBAS_FIX:
            fix.fix_type = "SBAS";
            break;
        default:
            fix.fix_type = "GPS";
            break;
    }

    return fix;
}

}  // namespace

IsaacbotLinkConfig IsaacbotLinkConfig::fromParameters(rclcpp::Node & node)
{
    IsaacbotLinkConfig config;

    config.isaacbot_amr_namespace = node.declare_parameter<std::string>("isaacbot.namespace", "");
    config.map_frame = resolveFrame(
        config.isaacbot_amr_namespace, node.declare_parameter<std::string>("isaacbot.map_frame", "map"));
    config.base_frame = resolveFrame(
        config.isaacbot_amr_namespace,
        node.declare_parameter<std::string>("isaacbot.base_frame", "base_link"));
    config.map_id = node.declare_parameter<std::string>("isaacbot.map_id", "");
    config.mission_prefix = node.declare_parameter<std::string>("isaacbot.mission_prefix", "vda5050");
    config.orientation_mode = parseOrientationMode(
        node.declare_parameter<std::string>("isaacbot.orientation_mode", "path"));

    const auto name = [&](const std::string & parameter, const std::string & default_name) {
        return resolveName(
            config.isaacbot_amr_namespace, node.declare_parameter<std::string>(parameter, default_name));
    };

    config.set_mission_service = name("isaacbot.set_mission_service", "set_mission");
    config.run_mission_service = name("isaacbot.run_mission_service", "run_mission");
    config.mission_state_topic = name("isaacbot.mission_state_topic", "mission_state");
    config.drive_mode_topic = name("isaacbot.drive_mode_topic", "drive_mode");
    config.drive_mode_service = name("isaacbot.drive_mode_service", "set_drive_mode");
    config.follow_me_start_service = name("isaacbot.follow_me_start_service", "follow_me/start");
    config.follow_me_stop_service = name("isaacbot.follow_me_stop_service", "follow_me/stop");
    config.battery_topic = name("isaacbot.battery_topic", "battery/battery_status");
    config.safety_status_topic =
        name("isaacbot.safety_status_topic", "hardware_interface/safety_status");
    config.motion_lock_topic = name("isaacbot.motion_lock_topic", "motion_lock");
    config.localization_state_topic =
        name("isaacbot.localization_state_topic", "localization_state");
    config.odom_topic = name("isaacbot.odom_topic", "odom");
    config.aux_output_service_prefix =
        name("isaacbot.aux_output_service_prefix", "hardware_interface/aux_output_");

    config.service_availability_timeout =
        node.declare_parameter<double>("isaacbot.service_availability_timeout", 2.0);
    config.service_response_timeout =
        node.declare_parameter<double>("isaacbot.service_response_timeout", 3.0);
    config.pose_timeout = node.declare_parameter<double>("isaacbot.pose_timeout", 2.0);
    config.status_timeout = node.declare_parameter<double>("isaacbot.status_timeout", 2.0);
    // The PLC acknowledges each write over Modbus; the drive UI allows the same 3 s.
    config.aux_output_timeout = node.declare_parameter<double>("isaacbot.aux_output_timeout", 3.0);

    // Delivery (isaacbot_amr_delivery_portal). NaN datum: off.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double datum_lat = node.declare_parameter<double>("isaacbot.geo.datum_lat", nan);
    const double datum_lon = node.declare_parameter<double>("isaacbot.geo.datum_lon", nan);
    if (domain::GeoPoint{datum_lat, datum_lon}.valid()) {
        config.datum = domain::GeoPoint{datum_lat, datum_lon};
    }
    config.enu_map_id = node.declare_parameter<std::string>("isaacbot.geo.enu_map_id", "enu_site");
    config.report_enu = node.declare_parameter<bool>("isaacbot.geo.report_enu", true);
    config.gps_fix_topic = name("isaacbot.geo.gps_fix_topic", "gps/fix");
    config.gps_course_topic = name("isaacbot.geo.gps_course_topic", "gps/heading_imu");
    config.start_mapping_service = name("isaacbot.geo.start_mapping_service", "start_mapping");
    config.start_trip_timeout = node.declare_parameter<double>("isaacbot.geo.start_trip_timeout", 60.0);
    config.no_fix_warning_s = node.declare_parameter<double>("isaacbot.geo.no_fix_warning_s", 30.0);
    config.global_costmap_node =
        name("isaacbot.geo.global_costmap_node", "global_costmap/global_costmap");
    // Compass bearing (0 = North, clockwise) the isaacbot faces when parked at the shop.
    const double parked_bearing_deg =
        node.declare_parameter<double>("isaacbot.geo.parked_heading_deg", nan);
    if (std::isfinite(parked_bearing_deg)) {
        config.geo.parked_heading_enu = (90.0 - parked_bearing_deg) * kDegToRad;
    }
    config.geo.parked_radius_m = node.declare_parameter<double>("isaacbot.geo.parked_radius_m", 30.0);
    config.geo.default_sigma_m = node.declare_parameter<double>("isaacbot.geo.default_sigma_m", 3.0);
    config.geo.aligner.min_step_m = node.declare_parameter<double>("isaacbot.geo.min_step_m", 2.0);
    config.geo.aligner.max_samples = static_cast<std::size_t>(
        node.declare_parameter<int>("isaacbot.geo.max_samples", 400));
    config.geo.aligner.min_spread_m =
        node.declare_parameter<double>("isaacbot.geo.min_spread_m", 15.0);
    config.geo.aligner.max_sigma_m = node.declare_parameter<double>("isaacbot.geo.max_sigma_m", 8.0);
    config.geo.aligner.outlier_gate_m =
        node.declare_parameter<double>("isaacbot.geo.outlier_gate_m", 4.0);
    config.realign.min_shift_m =
        node.declare_parameter<double>("isaacbot.geo.realign_min_shift_m", 1.0);
    config.realign.min_yaw_rad =
        node.declare_parameter<double>("isaacbot.geo.realign_min_yaw_deg", 3.0) * kDegToRad;

    // Cargo box: lock on aux output 1..6 (0 = none fitted).
    const auto lock_output = node.declare_parameter<int>("isaacbot.cargo.lock_output", 0);
    if (lock_output < 0 || lock_output > static_cast<int>(domain::kAuxOutputCount)) {
        throw std::invalid_argument("isaacbot.cargo.lock_output must be 0 (none) or 1..6");
    }
    if (lock_output > 0) {
        config.cargo.lock_output = lock_output - 1;
    }
    config.cargo.unlock_level = node.declare_parameter<bool>("isaacbot.cargo.unlock_level", true);
    config.pickup_topic = name("isaacbot.cargo.pickup_topic", "cargo/pickup_confirmed");
    config.pickup_timeout = node.declare_parameter<double>("isaacbot.cargo.pickup_timeout", 600.0);

    return config;
}

TfPoseSource::TfPoseSource(
    rclcpp::Node & node, std::string map_frame, std::string base_frame, double max_age)
: node_(node),
  map_frame_(std::move(map_frame)),
  base_frame_(std::move(base_frame)),
  max_age_(rclcpp::Duration::from_seconds(max_age)),
  buffer_(std::make_shared<tf2_ros::Buffer>(node.get_clock())),
  listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_, &node, true))
{
}

std::optional<domain::Pose2D> TfPoseSource::currentPose() const
{
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
        return std::nullopt;
    }

    // A chain whose dynamic links stopped publishing still resolves at its last common time.
    const rclcpp::Time stamp(transform.header.stamp, node_.get_clock()->get_clock_type());
    if (stamp.nanoseconds() != 0 && node_.now() - stamp > max_age_) {
        return std::nullopt;
    }

    return domain::Pose2D{
        transform.transform.translation.x, transform.transform.translation.y,
        yawOf(transform.transform.rotation)};
}

std::shared_ptr<IsaacbotLink> IsaacbotLink::forNode(rclcpp::Node * node)
{
    if (node == nullptr) {
        throw std::invalid_argument("IsaacbotLink needs the adapter node");
    }

    static std::mutex registry_mutex;
    static std::map<rclcpp::Node *, std::weak_ptr<IsaacbotLink>> registry;

    std::lock_guard<std::mutex> lock(registry_mutex);
    auto & entry = registry[node];

    if (auto link = entry.lock()) {
        return link;
    }

    auto link = std::make_shared<IsaacbotLink>(*node);
    entry = link;

    return link;
}

IsaacbotLink::IsaacbotLink(rclcpp::Node & node)
: node_(node),
  config_(IsaacbotLinkConfig::fromParameters(node)),
  safety_received_(0, 0, node.get_clock()->get_clock_type()),
  motion_lock_received_(0, 0, node.get_clock()->get_clock_type()),
  odom_received_(0, 0, node.get_clock()->get_clock_type()),
  fixes_from_(0, 0, node.get_clock()->get_clock_type()),
  fix_received_(0, 0, node.get_clock()->get_clock_type()),
  started_(node.now())
{
    using std::placeholders::_1;

    pose_source_ = std::make_shared<TfPoseSource>(
        node_, config_.map_frame, config_.base_frame, config_.pose_timeout);

    auto missions = std::make_shared<MissionManagerClient>(
        node_, config_.set_mission_service, config_.run_mission_service, config_.map_frame,
        std::chrono::duration<double>(config_.service_availability_timeout),
        std::chrono::duration<double>(config_.service_response_timeout));

    // The ENU frame normally arrives with each startTrip from the delivery portal; a datum in
    // the parameters only gives it a value before the first trip.
    std::optional<domain::EnuDatum> datum;
    if (config_.datum) {
        datum.emplace(*config_.datum, config_.enu_map_id);
    }
    geo_ = std::make_shared<application::GeoAlignmentUseCase>(datum, config_.geo);

    navigation_ = std::make_unique<application::NavigationUseCase>(
        missions, pose_source_, config_.mission_prefix, config_.orientation_mode, geo_,
        config_.realign);

    auto aux_client = std::make_shared<AuxOutputClient>(
        node_, config_.aux_output_service_prefix, domain::kAuxOutputCount,
        std::chrono::duration<double>(config_.service_availability_timeout),
        std::chrono::duration<double>(config_.aux_output_timeout));
    aux_outputs_ = std::make_unique<application::AuxOutputUseCase>(aux_client);
    cargo_ = std::make_unique<application::CargoUseCase>(aux_client, config_.cargo);

    drive_mode_use_case_ = std::make_unique<application::DriveModeUseCase>(
        std::make_shared<DriveModeClient>(
            node_, config_.drive_mode_service,
            std::chrono::duration<double>(config_.service_availability_timeout),
            std::chrono::duration<double>(config_.service_response_timeout)));

    follow_me_use_case_ = std::make_unique<application::FollowMeUseCase>(
        std::make_shared<FollowMeClient>(
            node_, config_.follow_me_start_service, config_.follow_me_stop_service,
            std::chrono::duration<double>(config_.service_availability_timeout),
            std::chrono::duration<double>(config_.service_response_timeout)));

    // QoS matches each publisher: the latched topics are transient-local depth 1.
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    mission_state_sub_ = node_.create_subscription<isaacbot_amr_msgs::msg::MissionState>(
        config_.mission_state_topic, latched,
        [this](const isaacbot_amr_msgs::msg::MissionState & msg) { onMissionState(msg); });

    drive_mode_sub_ = node_.create_subscription<isaacbot_amr_msgs::msg::DriveMode>(
        config_.drive_mode_topic, latched, [this](const isaacbot_amr_msgs::msg::DriveMode & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            drive_mode_ = msg;
        });

    localization_sub_ = node_.create_subscription<isaacbot_amr_msgs::msg::LocalizationState>(
        config_.localization_state_topic, latched,
        [this](const isaacbot_amr_msgs::msg::LocalizationState & msg) {
            using isaacbot_amr_msgs::msg::LocalizationState;
            bool new_session = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                localization_ = msg;
                // Another map is another frame, and so is a new SLAM session: the manager goes
                // through SWITCHING (or UNAVAILABLE) into MAPPING for each, but resumes a paused
                // one from ODOMETRY, which keeps its frame.
                const bool fresh_mapping = msg.mode == LocalizationState::MAPPING &&
                    (last_localization_mode_ == LocalizationState::SWITCHING ||
                     last_localization_mode_ == LocalizationState::UNAVAILABLE);
                new_session = fresh_mapping || msg.map_name != session_map_name_;
                session_map_name_ = msg.map_name;
                last_localization_mode_ = msg.mode;
            }
            if (new_session) {
                // Reset first, then count the session: startTrip must never see the new session
                // with the old session's alignment.
                if (geo_) {
                    geo_->reset();
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        fixes_from_ = node_.now() + rclcpp::Duration::from_seconds(kSessionSettle);
                    }
                    RCLCPP_INFO(node_.get_logger(), "New map frame: GPS alignment restarts.");
                }
                ++mapping_sessions_;
            }
        });

    battery_sub_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
        config_.battery_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::BatteryState & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            battery_ = msg;
        });

    safety_sub_ = node_.create_subscription<isaacbot_amr_msgs::msg::SafetyStatus>(
        config_.safety_status_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const isaacbot_amr_msgs::msg::SafetyStatus & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            safety_ = msg;
            safety_received_ = node_.now();
        });

    motion_lock_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        config_.motion_lock_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const std_msgs::msg::Bool & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_locked_ = msg.data;
            motion_lock_received_ = node_.now();
        });

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            odom_ = msg;
            odom_received_ = node_.now();
        });

    {
        // Reliable, as the GPS driver publishes (rules/ros2_communication.md: GNSS exception).
        fix_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(
            config_.gps_fix_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const sensor_msgs::msg::NavSatFix & msg) { onFix(msg); });

        course_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
            config_.gps_course_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
            [this](const sensor_msgs::msg::Imu & msg) {
                if (const auto pose = pose_source_->currentPose()) {
                    geo_->onCourse(yawOf(msg.orientation), *pose);
                }
            });

        if (config_.datum) {
            RCLCPP_INFO(
                node_.get_logger(),
                "Delivery ENU frame '%s' at %.7f, %.7f until a trip sets one (GPS for alignment "
                "only).", config_.enu_map_id.c_str(), config_.datum->lat, config_.datum->lon);
        }

        costmap_params_ =
            std::make_shared<rclcpp::AsyncParametersClient>(&node_, config_.global_costmap_node);

        // Without fixes no delivery can start (startTrip would only time out), and on the
        // indoor Nav 2 profile long trips choke the global costmap: say so, early and often.
        gps_watchdog_ = node_.create_wall_timer(
            std::chrono::duration<double>(std::max(1.0, config_.no_fix_warning_s)), [this]() {
                checkNavProfile();
                for (const auto & problem : {gpsProblem(), navProfileProblem()}) {
                    if (!problem.empty()) {
                        RCLCPP_WARN(node_.get_logger(), "%s", problem.c_str());
                    }
                }
            });
    }

    start_mapping_client_ =
        node_.create_client<std_srvs::srv::Trigger>(config_.start_mapping_service);

    pickup_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        config_.pickup_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::Bool & msg) {
            if (msg.data) {
                cargo_->onPickupConfirmed();
            }
        });

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "VDA 5050 isaacbot link: missions via '"
                                << config_.set_mission_service << "', poses in '"
                                << config_.map_frame << "' -> '" << config_.base_frame << "'.");
}

IsaacbotSnapshot IsaacbotLink::snapshot() const
{
    IsaacbotSnapshot snapshot;
    snapshot.pose = pose_source_->currentPose();
    snapshot.paused = navigation_->paused();
    snapshot.health.mission_refusal = navigation_->lastRefusal();

    if (geo_) {
        snapshot.enu_map_id = geo_->mapId();
        snapshot.report_enu = config_.report_enu;
        snapshot.gps_fix = geo_->latestFix();
        snapshot.alignment = geo_->quality();
        if (snapshot.pose) {
            snapshot.enu_pose = geo_->toEnu(*snapshot.pose);
        }
    }
    const double geo_score = geo_ ? geo_->score() : 0.0;
    snapshot.gps_problem = gpsProblem();
    snapshot.nav_profile_problem = navProfileProblem();

    // Taken only after the use cases' own locks are released: they never nest.
    std::lock_guard<std::mutex> lock(mutex_);

    snapshot.map_id = config_.map_id;
    if (localization_ && !localization_->map_name.empty()) {
        snapshot.map_id = localization_->map_name;
    }
    snapshot.localization_score = snapshot.enu_pose ? geo_score : (snapshot.pose ? 1.0 : 0.0);

    // A fix the receiver stopped sending is not shown as current.
    if (snapshot.gps_fix && !fresh(fix_received_)) {
        snapshot.gps_fix.reset();
    }

    if (odom_ && fresh(odom_received_)) {
        snapshot.velocity.vx = odom_->twist.twist.linear.x;
        snapshot.velocity.vy = odom_->twist.twist.linear.y;
        snapshot.velocity.omega = odom_->twist.twist.angular.z;
    }

    if (battery_) {
        snapshot.battery = batteryReadingFor(*battery_);
    }

    auto & health = snapshot.health;
    health.pose_known = snapshot.pose.has_value();

    if (drive_mode_) {
        health.drive_mode = driveModeFor(drive_mode_->mode);
        health.guard = guardFor(drive_mode_->guard);
    }

    if (safety_) {
        health.safety = domain::SafetyReading{
            safety_->hw_e_stop_user_button, safety_->latch_active,
            // Stale data is as untrustworthy as data the PLC link itself flags.
            safety_->link_healthy && fresh(safety_received_)};
    }

    if (motion_locked_ && fresh(motion_lock_received_)) {
        health.motion_locked = *motion_locked_;
    }

    return snapshot;
}

void IsaacbotLink::setProgressListener(std::function<void()> listener)
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_listener_ = std::move(listener);
}

void IsaacbotLink::clearProgressListener()
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_listener_ = nullptr;
}

void IsaacbotLink::onMissionState(const isaacbot_amr_msgs::msg::MissionState & msg)
{
    navigation_->onMissionProgress(
        domain::MissionProgress{
            msg.mission_id, phaseFor(msg.state), msg.current_index, msg.total, msg.message});

    std::function<void()> listener;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        listener = progress_listener_;
    }

    if (listener) {
        listener();
    }
}

void IsaacbotLink::onFix(const sensor_msgs::msg::NavSatFix & msg)
{
    const auto fix = gpsFixFor(msg);
    if (!fix) {
        return;
    }

    bool settled = true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fix_received_ = node_.now();
        settled = fix_received_ >= fixes_from_;
    }

    // While a new SLAM session settles, the fix is shown but not paired with a pose.
    geo_->onFix(*fix, settled ? pose_source_->currentPose() : std::nullopt);
}

application::CommandResult IsaacbotLink::startMapping()
{
    auto & client = *start_mapping_client_;
    const std::string service = client.get_service_name();

    if (!client.wait_for_service(
            std::chrono::duration<double>(config_.service_availability_timeout)))
    {
        return {false, "'" + service + "' unavailable (isaacbot_amr_indoor_nav_manager).", false};
    }

    auto future = client.async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::duration<double>(config_.service_response_timeout)) !=
        std::future_status::ready)
    {
        client.remove_pending_request(future);
        return {false, "'" + service + "' did not answer in time.", true};
    }

    const auto response = future.get();
    return {response->success, response->message, false};
}

std::string IsaacbotLink::gpsProblem() const
{
    // Only a delivery isaacbot (it knows a datum, from a trip or its parameters) needs GPS.
    const auto datum = geo_ ? geo_->datum() : std::nullopt;
    if (!datum) {
        return {};
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = node_.now();

    if (fix_received_.nanoseconds() == 0) {
        if ((now - started_).seconds() < config_.no_fix_warning_s) {
            return {};
        }
        return "Delivery is set up (datum " + std::to_string(datum->origin().lat) + ", " +
               std::to_string(datum->origin().lon) + ") but no GPS fix has arrived on '" +
               config_.gps_fix_topic +
               "': is ISAACBOT_AMR_SYSTEM_USE_GPS=true (it starts the GNSS driver) and does the "
               "receiver have a fix? Without one no delivery trip can start.";
    }

    if ((now - fix_received_).seconds() >= config_.no_fix_warning_s) {
        return "No GPS fix for " + std::to_string(static_cast<int>((now - fix_received_).seconds())) +
               " s on '" + config_.gps_fix_topic + "' (lost fix or GNSS driver stopped): new "
               "delivery trips cannot start; a running one keeps its last alignment.";
    }

    return {};
}

void IsaacbotLink::checkNavProfile()
{
    // Executor thread: asynchronous, the answer lands in global_rolling_ later.
    if (!costmap_params_ || !costmap_params_->service_is_ready()) {
        return;
    }
    costmap_params_->get_parameters(
        {"rolling_window"},
        [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
            try {
                const auto params = future.get();
                if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    global_rolling_ = params[0].as_bool();
                }
            } catch (const std::exception &) {
                // Nav 2 restarting: ask again next time.
            }
        });
}

std::string IsaacbotLink::navProfileProblem() const
{
    const bool delivery = geo_ && geo_->datum().has_value();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!delivery || !global_rolling_ || *global_rolling_) {
        return {};
    }
    return "This isaacbot takes delivery trips, but Nav 2 runs the indoor profile: its global "
           "costmap grows with the live map and long trips will slow it down or stall it. Set "
           "ISAACBOT_AMR_ORCH_NAV_PROFILE=delivery on this device.";
}

bool IsaacbotLink::readyForEnuOrders() const
{
    if (!geo_ || !geo_->enuToNavigation() || !pose_source_->currentPose()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    return localization_ && localization_->mode == isaacbot_amr_msgs::msg::LocalizationState::MAPPING;
}

domain::RouteFrame IsaacbotLink::frameOf(const std::string & map_id) const
{
    // Recognised even without a datum: such an order is then refused, never driven as raw
    // SLAM-map metres.
    const auto trip_map_id = geo_ ? geo_->mapId() : std::string();
    return !map_id.empty() && (map_id == trip_map_id || map_id == config_.enu_map_id) ?
        domain::RouteFrame::kEnu : domain::RouteFrame::kNavigation;
}

void IsaacbotLink::configureDelivery(
    const domain::EnuDatum & datum, std::optional<double> parked_heading_enu)
{
    geo_->configure(datum, parked_heading_enu);
    RCLCPP_INFO(
        node_.get_logger(), "Delivery ENU frame '%s' at %.7f, %.7f, parked heading %s (from the trip).",
        datum.mapId().c_str(), datum.origin().lat, datum.origin().lon,
        parked_heading_enu ? (std::to_string(90.0 - *parked_heading_enu / kDegToRad) + " deg").c_str()
                           : "unknown");
}

bool IsaacbotLink::fresh(const rclcpp::Time & received) const
{
    return received.nanoseconds() != 0 &&
           (node_.now() - received).seconds() <= config_.status_timeout;
}

}  // namespace isaacbot_amr_vda5050_adapter::infrastructure
