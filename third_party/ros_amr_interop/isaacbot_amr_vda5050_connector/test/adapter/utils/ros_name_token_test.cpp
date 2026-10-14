// BSD 3-Clause License
//
// Copyright (c) 2022 InOrbit, Inc.
// Copyright (c) 2022 Clearpath Robotics, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the InOrbit, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <string>

#include "isaacbot_amr_vda5050_connector/utils.hpp"
#include "rcl/validate_topic_name.h"

using isaacbot_amr_vda5050_connector::utils::ros_name_token;

// isaacbot_amr_vda5050: same cases as test_ros_name_token in test/test_utils.py, so the adapter
// (C++) and the controller (Python) build identical ROS names.
TEST(RosNameTokenTest, MatchesPythonCases)
{
  EXPECT_EQ(ros_name_token("ardennes"), "ardennes");
  EXPECT_EQ(ros_name_token("MechatronicsAcademy"), "MechatronicsAcademy");
  EXPECT_EQ(ros_name_token("rover-001"), "rover_001");
  EXPECT_EQ(ros_name_token("a.b:c"), "a_b_c");
  EXPECT_EQ(ros_name_token("0001"), "_0001");
  EXPECT_EQ(ros_name_token(""), "_");
}

TEST(RosNameTokenTest, BuildsValidTopicNameFromVdaSerial)
{
  const std::string name = "/ardennes_001/vda5050/" + ros_name_token("Mechatronics-Academy") +
    "/" + ros_name_token("rover-001") + "/adapter/nav_through_nodes";
  int result = -1;
  size_t invalid_index = 0;
  ASSERT_EQ(rcl_validate_topic_name(name.c_str(), &result, &invalid_index), RCL_RET_OK);
  EXPECT_EQ(result, RCL_TOPIC_NAME_VALID) << rcl_topic_name_validation_result_string(result);
}
