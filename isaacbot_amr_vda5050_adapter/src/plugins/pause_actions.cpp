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

#include "pluginlib/class_list_macros.hpp"

#include "isaacbot_amr_vda5050_adapter/plugins/instant_isaacbot_amr_action.hpp"

namespace isaacbot_amr_vda5050_adapter::plugins
{

/** @brief VDA 5050 startPause: stop driving and keep the order. */
class StartPause : public InstantIsaacbotAction
{
protected:
    application::CommandResult command() override { return link_->navigation().pause(); }
};

/** @brief VDA 5050 stopPause: continue the order from the first node not yet reached. */
class StopPause : public InstantIsaacbotAction
{
protected:
    application::CommandResult command() override { return link_->navigation().resume(); }
};

}  // namespace isaacbot_amr_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::StartPause, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::StopPause, adapter::VDAAction)
