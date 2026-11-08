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


#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "isaacbot_amr_vda5050_adapter/application/cargo_use_case.hpp"

using isaacbot_amr_vda5050_adapter::application::AuxOutputPort;
using isaacbot_amr_vda5050_adapter::application::CargoConfig;
using isaacbot_amr_vda5050_adapter::application::CargoUseCase;
using isaacbot_amr_vda5050_adapter::application::CommandResult;

namespace
{

class FakeAuxOutputs : public AuxOutputPort
{
public:
    CommandResult set(int index, bool enabled) override
    {
        writes.emplace_back(index, enabled);
        return result;
    }

    std::vector<std::pair<int, bool>> writes;
    CommandResult result{true, "", false};
};

}  // namespace

TEST(CargoUseCase, UnlocksAndLocksTheConfiguredOutput)
{
    auto outputs = std::make_shared<FakeAuxOutputs>();
    CargoUseCase cargo(outputs, CargoConfig{2, true});

    EXPECT_EQ(cargo.open().message, "Cargo box unlocked.");
    EXPECT_EQ(cargo.close().message, "Cargo box locked.");
    ASSERT_EQ(outputs->writes.size(), 2U);
    EXPECT_EQ(outputs->writes[0], std::make_pair(2, true));
    EXPECT_EQ(outputs->writes[1], std::make_pair(2, false));
}

TEST(CargoUseCase, ActiveLowLock)
{
    auto outputs = std::make_shared<FakeAuxOutputs>();
    CargoUseCase cargo(outputs, CargoConfig{0, false});
    cargo.open();
    EXPECT_EQ(outputs->writes[0], std::make_pair(0, false));
}

TEST(CargoUseCase, NoLockFittedSucceedsWithoutSwitching)
{
    CargoUseCase cargo(nullptr, CargoConfig{});
    EXPECT_TRUE(cargo.open().ok);
    EXPECT_TRUE(cargo.close().ok);
}

TEST(CargoUseCase, ReportsAFailedWrite)
{
    auto outputs = std::make_shared<FakeAuxOutputs>();
    outputs->result = {false, "PLC timeout", true};
    CargoUseCase cargo(outputs, CargoConfig{1, true});
    const auto result = cargo.open();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Could not unlock the cargo box: PLC timeout");
}

TEST(CargoUseCase, CountsPickupConfirmations)
{
    CargoUseCase cargo(nullptr, CargoConfig{});
    const auto before = cargo.confirmations();
    cargo.onPickupConfirmed();
    EXPECT_EQ(cargo.confirmations(), before + 1);
}
