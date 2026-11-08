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


#include "isaacbot_amr_vda5050_adapter/application/cargo_use_case.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace isaacbot_amr_vda5050_adapter::application
{

CargoUseCase::CargoUseCase(std::shared_ptr<AuxOutputPort> outputs, CargoConfig config)
: outputs_(std::move(outputs)), config_(config)
{
    if (config_.lock_output && outputs_ == nullptr) {
        throw std::invalid_argument("CargoUseCase needs an AuxOutputPort for its lock output");
    }
}

CommandResult CargoUseCase::open()
{
    return setLocked(false);
}

CommandResult CargoUseCase::close()
{
    return setLocked(true);
}

CommandResult CargoUseCase::setLocked(bool locked)
{
    if (!config_.lock_output) {
        return {true, locked ? "No cargo lock fitted; nothing to lock." :
                               "No cargo lock fitted; nothing to unlock.", false};
    }

    const bool level = locked ? !config_.unlock_level : config_.unlock_level;
    auto result = outputs_->set(*config_.lock_output, level);

    if (result.ok) {
        result.message = locked ? "Cargo box locked." : "Cargo box unlocked.";
    } else {
        result.message = std::string(locked ? "Could not lock" : "Could not unlock") +
                         " the cargo box: " + result.message;
    }

    return result;
}

}  // namespace isaacbot_amr_vda5050_adapter::application
