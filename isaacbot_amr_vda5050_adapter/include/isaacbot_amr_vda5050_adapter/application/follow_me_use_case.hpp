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

#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_

#include <memory>
#include <mutex>

#include "isaacbot_amr_vda5050_adapter/application/ports.hpp"

namespace isaacbot_amr_vda5050_adapter::application
{

/**
 * @brief startFollowing / stopFollowing: fleet control starts or stops follow-me.
 *
 * The isaacbot's follow_me node stays the authority: it refuses a start outside Automatic, during a
 * mission or with nobody in front of the isaacbot, and its reason becomes the action's result.
 * Following then runs outside VDA 5050 (Nav 2's Following server), so the action finishes as soon
 * as following has started or stopped; a later order stops following on the isaacbot side. Blocks
 * on FollowMePort: never call it from the executor thread.
 */
class FollowMeUseCase
{
public:
    explicit FollowMeUseCase(std::shared_ptr<FollowMePort> follow_me);

    CommandResult start();
    CommandResult stop();

private:
    std::shared_ptr<FollowMePort> follow_me_;
    std::mutex mutex_;
};

}  // namespace isaacbot_amr_vda5050_adapter::application

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_
