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


#ifndef ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_CARGO_USE_CASE_HPP_
#define ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_CARGO_USE_CASE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "isaacbot_amr_vda5050_adapter/application/ports.hpp"

namespace isaacbot_amr_vda5050_adapter::application
{

struct CargoConfig
{
    /// The aux output (0-based, 0 = DIO00) driving the cargo box lock; none: no lock fitted, and
    /// openCargo / closeCargo succeed without switching anything.
    std::optional<int> lock_output;
    /// The output level that unlocks the box (a fail-secure solenoid unlocks when energized).
    bool unlock_level{true};
};

/**
 * @brief The delivery cargo box: lock, unlock, and the customer's pickup confirmation.
 *
 * The hardware is not chosen yet, so the box is only a lock on an aux output plus a "picked up"
 * signal that anything may raise: a lid switch, a PIN keypad node, or an operator
 * (`ros2 topic pub --once cargo/pickup_confirmed std_msgs/Bool "{data: true}"`).
 */
class CargoUseCase
{
public:
    CargoUseCase(std::shared_ptr<AuxOutputPort> outputs, CargoConfig config);

    /** @brief openCargo. Blocks on the PLC write (never on the executor thread). */
    CommandResult open();
    /** @brief closeCargo. Blocks on the PLC write (never on the executor thread). */
    CommandResult close();

    /** @brief The customer took the order (any thread). */
    void onPickupConfirmed() { ++confirmations_; }
    /** @brief Confirmations so far: confirmPickup waits for this to grow. */
    std::uint64_t confirmations() const { return confirmations_.load(); }

private:
    CommandResult setLocked(bool locked);

    std::shared_ptr<AuxOutputPort> outputs_;
    CargoConfig config_;
    std::atomic<std::uint64_t> confirmations_{0};
};

}  // namespace isaacbot_amr_vda5050_adapter::application

#endif  // ISAACBOT_AMR_VDA5050_ADAPTER_APPLICATION_CARGO_USE_CASE_HPP_
