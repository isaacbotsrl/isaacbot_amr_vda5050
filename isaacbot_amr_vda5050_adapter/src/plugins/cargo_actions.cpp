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
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "isaacbot_amr_vda5050_connector/vda_action.hpp"

#include "isaacbot_amr_vda5050_adapter/infrastructure/isaacbot_amr_link.hpp"
#include "isaacbot_amr_vda5050_adapter/plugins/instant_isaacbot_amr_action.hpp"

namespace isaacbot_amr_vda5050_adapter::plugins
{

/** @brief openCargo: unlock the delivery cargo box (destination node of a delivery order). */
class OpenCargo : public InstantIsaacbotAction
{
protected:
    application::CommandResult command() override { return link_->cargo().open(); }
};

/** @brief closeCargo: lock the delivery cargo box. */
class CloseCargo : public InstantIsaacbotAction
{
protected:
    application::CommandResult command() override { return link_->cargo().close(); }
};

/**
 * @brief confirmPickup: wait until the customer took the order (cargo/pickup_confirmed).
 *
 * A HARD-blocking node action: the isaacbot stays at the customer until the signal comes,
 * isaacbot.cargo.pickup_timeout passes (FAILED), or master control cancels the order (FAILED).
 * Only a confirmation raised after the action started counts.
 */
class ConfirmPickup : public adapter::VDAAction
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
        start_count_ = link_->cargo().confirmations();
        deadline_ = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(link_->config().pickup_timeout));
        current_action_msg_.result_description = "Waiting for the customer.";
        update_action_state(STATES::RUNNING);
    }

    void run() override
    {
        // Runs on the connector's action thread: waiting here blocks nothing else.
        while (true) {
            if (link_->cargo().confirmations() > start_count_) {
                current_action_msg_.result_description = "The customer took the order.";
                update_action_state(STATES::FINISHED);
                return;
            }
            if (cancelled_) {
                current_action_msg_.result_description = "Cancelled before the pickup.";
                update_action_state(STATES::FAILED);
                return;
            }
            if (std::chrono::steady_clock::now() >= deadline_) {
                current_action_msg_.result_description = "Nobody picked the order up in time.";
                update_action_state(STATES::FAILED);
                return;
            }
            std::this_thread::sleep_for(kPollPeriod);
        }
    }

    bool cancel() override
    {
        // Executor thread: only flag it; run() ends the action on its own thread.
        cancelled_ = true;
        return true;
    }

    void finish() override { goal_handle_->succeed(result_); }
    void fail() override { goal_handle_->abort(result_); }

private:
    static constexpr std::chrono::milliseconds kPollPeriod{200};

    std::shared_ptr<infrastructure::IsaacbotLink> link_;
    std::atomic<bool> cancelled_{false};
    std::uint64_t start_count_{0};
    std::chrono::steady_clock::time_point deadline_;
};

}  // namespace isaacbot_amr_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::OpenCargo, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::CloseCargo, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(isaacbot_amr_vda5050_adapter::plugins::ConfirmPickup, adapter::VDAAction)
