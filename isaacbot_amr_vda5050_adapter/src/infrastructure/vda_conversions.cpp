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

#include "isaacbot_amr_vda5050_adapter/infrastructure/vda_conversions.hpp"

#include <cstdio>
#include <string>

namespace isaacbot_amr_vda5050_adapter::infrastructure
{

std::vector<domain::RouteNode> toRouteNodes(const std::vector<isaacbot_amr_vda5050_msgs::msg::Node> & nodes)
{
    std::vector<domain::RouteNode> route;
    route.reserve(nodes.size());

    for (const auto & node : nodes) {
        route.push_back(
            domain::RouteNode{
                node.node_id, node.sequence_id,
                domain::Pose2D{
                    node.node_position.x, node.node_position.y, node.node_position.theta}});
    }

    return route;
}

isaacbot_amr_vda5050_msgs::msg::AGVPosition toAgvPosition(const IsaacbotSnapshot & snapshot)
{
    isaacbot_amr_vda5050_msgs::msg::AGVPosition position;
    const bool enu = snapshot.report_enu && snapshot.enu_pose.has_value();
    const auto & pose = enu ? snapshot.enu_pose : snapshot.pose;

    position.map_id = enu ? snapshot.enu_map_id : snapshot.map_id;
    position.position_initialized = pose.has_value();
    position.localization_score = snapshot.localization_score;

    if (pose) {
        position.x = pose->x;
        position.y = pose->y;
        position.theta = pose->theta;
    }

    return position;
}

namespace
{

isaacbot_amr_vda5050_msgs::msg::InfoReference reference(const std::string & key, const std::string & value)
{
    isaacbot_amr_vda5050_msgs::msg::InfoReference ref;
    ref.reference_key = key;
    ref.reference_value = value;
    return ref;
}

std::string number(double value, int decimals)
{
    char buffer[48];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return buffer;
}

}  // namespace

std::vector<isaacbot_amr_vda5050_msgs::msg::Info> toInformation(const IsaacbotSnapshot & snapshot)
{
    using isaacbot_amr_vda5050_msgs::msg::Info;

    std::vector<Info> infos;

    if (snapshot.gps_fix) {
        Info info;
        info.info_type = "gpsFix";
        info.info_level = Info::INFO;
        info.info_description = "Raw GNSS fix (display only; the isaacbot navigates by lidar).";
        info.info_references = {
            reference("lat", number(snapshot.gps_fix->point.lat, 8)),
            reference("lon", number(snapshot.gps_fix->point.lon, 8)),
            reference("fixType", snapshot.gps_fix->fix_type),
            reference("sigma", number(snapshot.gps_fix->sigma_m, 2))};
        infos.push_back(info);
    }

    if (snapshot.alignment) {
        Info info;
        info.info_type = "alignment";
        info.info_level = Info::DEBUG;
        info.info_description = "GPS (ENU) to SLAM map alignment.";
        info.info_references = {
            reference("aligned", snapshot.enu_pose ? "true" : "false"),
            reference("rmsM", number(snapshot.alignment->rms_m, 2)),
            reference("samples", std::to_string(snapshot.alignment->samples)),
            reference("spreadM", number(snapshot.alignment->spread_m, 1)),
            reference("yawFromFit", snapshot.alignment->yaw_from_fit ? "true" : "false")};
        infos.push_back(info);
    }

    return infos;
}

isaacbot_amr_vda5050_msgs::msg::Velocity toVelocity(const IsaacbotSnapshot & snapshot)
{
    isaacbot_amr_vda5050_msgs::msg::Velocity velocity;
    velocity.vx = snapshot.velocity.vx;
    velocity.vy = snapshot.velocity.vy;
    velocity.omega = snapshot.velocity.omega;

    return velocity;
}

isaacbot_amr_vda5050_msgs::msg::BatteryState toBatteryState(const domain::VdaBattery & battery)
{
    isaacbot_amr_vda5050_msgs::msg::BatteryState state;
    state.battery_charge = battery.charge_percent;
    state.battery_voltage = battery.voltage;
    state.charging = battery.charging;

    return state;
}

isaacbot_amr_vda5050_msgs::msg::SafetyState toSafetyState(const domain::VdaSafety & safety)
{
    isaacbot_amr_vda5050_msgs::msg::SafetyState state;
    state.e_stop = safety.e_stop;
    state.field_violation = safety.field_violation;

    return state;
}

isaacbot_amr_vda5050_msgs::msg::Error toError(const domain::IsaacbotError & error)
{
    isaacbot_amr_vda5050_msgs::msg::Error msg;
    msg.error_type = error.type;
    msg.error_description = error.description;
    msg.error_level = error.fatal ? isaacbot_amr_vda5050_msgs::msg::Error::FATAL :
                                    isaacbot_amr_vda5050_msgs::msg::Error::WARNING;

    return msg;
}

}  // namespace isaacbot_amr_vda5050_adapter::infrastructure
