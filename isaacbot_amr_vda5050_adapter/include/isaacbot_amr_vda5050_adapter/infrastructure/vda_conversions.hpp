// Copyright 2026 Mechatronics Academy
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

#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_

#include <vector>

#include "isaacbot_amr_vda5050_msgs/msg/agv_position.hpp"
#include "isaacbot_amr_vda5050_msgs/msg/battery_state.hpp"
#include "isaacbot_amr_vda5050_msgs/msg/error.hpp"
#include "isaacbot_amr_vda5050_msgs/msg/node.hpp"
#include "isaacbot_amr_vda5050_msgs/msg/safety_state.hpp"
#include "isaacbot_amr_vda5050_msgs/msg/velocity.hpp"

#include "isaacbot_amr_vda5050_adapter/domain/isaacbot_amr_status.hpp"
#include "isaacbot_amr_vda5050_adapter/domain/types.hpp"
#include "isaacbot_amr_vda5050_adapter/infrastructure/isaacbot_amr_link.hpp"

namespace isaacbot_amr_vda5050_adapter::infrastructure
{

/// Order nodes as route nodes. VDA 5050 positions are taken to be in the navigation frame.
std::vector<domain::RouteNode> toRouteNodes(const std::vector<isaacbot_amr_vda5050_msgs::msg::Node> & nodes);

isaacbot_amr_vda5050_msgs::msg::AGVPosition toAgvPosition(const IsaacbotSnapshot & snapshot);
isaacbot_amr_vda5050_msgs::msg::Velocity toVelocity(const IsaacbotSnapshot & snapshot);
isaacbot_amr_vda5050_msgs::msg::BatteryState toBatteryState(const domain::VdaBattery & battery);
isaacbot_amr_vda5050_msgs::msg::SafetyState toSafetyState(const domain::VdaSafety & safety);
isaacbot_amr_vda5050_msgs::msg::Error toError(const domain::IsaacbotError & error);

}  // namespace isaacbot_amr_vda5050_adapter::infrastructure

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_
