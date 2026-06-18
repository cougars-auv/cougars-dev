// Copyright (c) 2026 BYU FROST Lab
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

/**
 * @file test_status_codec.cpp
 * @brief Unit tests for status_codec.hpp.
 * @author Nelson Durrant (w Opus 4.8)
 * @date June 2026
 */

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "coug_comms/utils/status_codec.hpp"

using coug_comms::utils::decodeStatus;
using coug_comms::utils::encodeStatus;
using coug_comms::utils::kStatusPacketLen;
using coug_comms::utils::MsgId;
using coug_interfaces::msg::AgentStatus;

/**
 * @brief Verify every transmitted field survives an encode/decode round-trip.
 */
TEST(StatusCodecTest, RoundTrip) {
  auto quatAngleDeg = [](const geometry_msgs::msg::Quaternion& a,
                         const geometry_msgs::msg::Quaternion& b) {
    double dot = std::min(1.0, std::fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w));
    return 2.0 * std::acos(dot) * 180.0 / M_PI;
  };

  AgentStatus in;
  in.local_odometry.position.x = 12.34;
  in.local_odometry.position.y = -56.78;
  in.local_odometry.position.z = 3.20;
  const double n = std::sqrt(0.1 * 0.1 + 0.2 * 0.2 + 0.3 * 0.3 + 0.9 * 0.9);
  in.local_odometry.orientation.x = 0.1 / n;
  in.local_odometry.orientation.y = 0.2 / n;
  in.local_odometry.orientation.z = 0.3 / n;
  in.local_odometry.orientation.w = 0.9 / n;
  in.imu_orientation.z = std::sin(0.3);
  in.imu_orientation.w = std::cos(0.3);
  in.pressure_depth = 4.05;
  for (int i = 0; i < 6; ++i) in.odometry_covariance[7 * i] = 0.01 * (i + 1);

  std::array<uint8_t, 30> buf{};
  const uint8_t len = encodeStatus(in, buf.data());
  ASSERT_EQ(len, kStatusPacketLen);
  ASSERT_LE(len, 30u);
  EXPECT_EQ(buf[0], static_cast<uint8_t>(MsgId::MSG_ORIGIN));

  AgentStatus out;
  ASSERT_TRUE(decodeStatus(buf.data(), len, out));

  EXPECT_NEAR(out.local_odometry.position.x, in.local_odometry.position.x, 0.005);
  EXPECT_NEAR(out.local_odometry.position.y, in.local_odometry.position.y, 0.005);
  EXPECT_NEAR(out.local_odometry.position.z, in.local_odometry.position.z, 0.005);
  EXPECT_NEAR(out.pressure_depth, in.pressure_depth, 0.005);
  EXPECT_LT(quatAngleDeg(out.local_odometry.orientation, in.local_odometry.orientation), 0.3);
  EXPECT_LT(quatAngleDeg(out.imu_orientation, in.imu_orientation), 0.3);
  for (int i = 0; i < 6; ++i) {
    const double expected = in.odometry_covariance[7 * i];
    EXPECT_NEAR(out.odometry_covariance[7 * i], expected, expected * 0.01);
  }
}

/**
 * @brief Verify only the covariance diagonal is kept; off-diagonals decode to zero.
 */
TEST(StatusCodecTest, DropsOffDiagonalCovariance) {
  AgentStatus in;
  in.odometry_covariance[1] = 99.0;
  in.odometry_covariance[6] = 99.0;

  std::array<uint8_t, 30> buf{};
  const uint8_t len = encodeStatus(in, buf.data());
  AgentStatus out;
  ASSERT_TRUE(decodeStatus(buf.data(), len, out));

  EXPECT_DOUBLE_EQ(out.odometry_covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(out.odometry_covariance[6], 0.0);
}

/**
 * @brief Verify decoding fails on a short payload or a wrong discriminator byte.
 */
TEST(StatusCodecTest, RejectsInvalidPayload) {
  std::array<uint8_t, 30> buf{};
  const uint8_t len = encodeStatus(AgentStatus(), buf.data());
  AgentStatus out;

  EXPECT_FALSE(decodeStatus(buf.data(), len - 1, out));
  buf[0] = static_cast<uint8_t>(MsgId::SRV_START);
  EXPECT_FALSE(decodeStatus(buf.data(), len, out));
}
