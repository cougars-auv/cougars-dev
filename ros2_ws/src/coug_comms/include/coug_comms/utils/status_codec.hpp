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
 * @file status_codec.hpp
 * @brief Compact acoustic encoding of coug_interfaces/AgentStatus.
 * @author Nelson Durrant (w Opus 4.8)
 * @date June 2026
 *
 * AgentStatus is too large for a Seatrac DAT packet (max 30 bytes), so only the
 * navigation summary is quantized onto the wire. The 29-byte layout:
 *
 *   offset  field                              encoding
 *   ------  ---------------------------------  -----------------------------
 *    0      MsgId::MSG_ORIGIN discriminator    uint8
 *    1- 6   local_odometry.position (x,y,z)    int16 x3, 1 cm (+/- 327.67 m)
 *    7-10   local_odometry.orientation         smallest-three quaternion
 *   11-22   odometry_covariance diagonal (6)   float16 x6
 *   23-24   pressure_depth                     int16, 1 cm
 *   25-28   imu_orientation                    smallest-three quaternion
 *
 * The header and off-diagonal covariance are not sent; multi-byte fields are
 * little-endian.
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "coug_comms/utils/comms_protocol.hpp"
#include "coug_interfaces/msg/agent_status.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace coug_comms::utils {

/**
 * @brief Encoded wire size of an AgentStatus, in bytes (fits the 30-byte DAT payload).
 */
inline constexpr uint8_t kStatusPacketLen = 29;

/**
 * @brief Writes a little-endian value to a buffer and advances the offset.
 * @param buf The destination buffer.
 * @param off The write offset, advanced by sizeof(T).
 * @param value The value to write.
 */
template <typename T>
inline void writeLE(uint8_t* buf, size_t& off, T value) {
  std::memcpy(buf + off, &value, sizeof(T));
  off += sizeof(T);
}

/**
 * @brief Reads a little-endian value from a buffer and advances the offset.
 * @param buf The source buffer.
 * @param off The read offset, advanced by sizeof(T).
 * @return The decoded value.
 */
template <typename T>
inline T readLE(const uint8_t* buf, size_t& off) {
  T value;
  std::memcpy(&value, buf + off, sizeof(T));
  off += sizeof(T);
  return value;
}

/**
 * @brief Fixed-point scale for distances, where one count is one centimeter.
 */
inline constexpr double kMetersScale = 100.0;

/**
 * @brief Quantizes a distance in meters to a 1 cm signed 16-bit fixed point.
 * @param meters The distance in meters (clamped to +/- 327.67 m).
 * @return The quantized value.
 */
inline int16_t encodeMeters(double meters) {
  const long counts = std::clamp<long>(std::lround(meters * kMetersScale), -32768, 32767);
  return static_cast<int16_t>(counts);
}

/**
 * @brief Restores a distance in meters from its 1 cm fixed-point encoding.
 * @param counts The quantized value.
 * @return The distance in meters.
 */
inline double decodeMeters(int16_t counts) { return static_cast<double>(counts) / kMetersScale; }

/**
 * @brief Converts a float to an IEEE-754 half-precision bit pattern.
 * @param value The value to convert.
 * @return The 16-bit half-precision bits.
 */
inline uint16_t floatToHalf(float value) {
  return Eigen::numext::bit_cast<uint16_t>(Eigen::half(value));
}

/**
 * @brief Converts an IEEE-754 half-precision bit pattern back to a float.
 * @param bits The 16-bit half-precision bits.
 * @return The decoded value.
 */
inline float halfToFloat(uint16_t bits) {
  return static_cast<float>(Eigen::numext::bit_cast<Eigen::half>(bits));
}

inline constexpr double kInvSqrt2 = 0.70710678118654752440;
inline constexpr int kQuatBits = 10;                           // bits per component
inline constexpr int kSelectorShift = 3 * kQuatBits;           // selector, above the 3 components
inline constexpr uint32_t kQuatMask = (1u << kQuatBits) - 1;   // component mask
inline constexpr long kQuatMax = (1L << (kQuatBits - 1)) - 1;  // max signed magnitude

/**
 * @brief Packs a unit quaternion into 4 bytes using the smallest-three method.
 *
 * Drops the largest component (recomputed at decode from the unit-norm
 * constraint) and stores its 2-bit index plus the other three as 10-bit signed
 * values over [-1/sqrt(2), 1/sqrt(2)], flipped so the dropped one is positive.
 *
 * @param quat The quaternion to pack (need not be normalized).
 * @return The packed 32-bit word.
 */
inline uint32_t packQuaternion(const geometry_msgs::msg::Quaternion& quat) {
  double q[4] = {quat.x, quat.y, quat.z, quat.w};
  double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm < 1e-9) {  // degenerate input -> identity rotation
    q[0] = q[1] = q[2] = 0.0;
    q[3] = norm = 1.0;
  }
  for (double& c : q) c /= norm;

  int largest = 0;
  for (int i = 1; i < 4; ++i) {
    if (std::fabs(q[i]) > std::fabs(q[largest])) largest = i;
  }
  const double sign = (q[largest] < 0.0) ? -1.0 : 1.0;  // force dropped component positive

  uint32_t packed = static_cast<uint32_t>(largest) << kSelectorShift;
  int shift = 2 * kQuatBits;  // remaining components at bit 20, 10, then 0
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    const long value =
        std::clamp<long>(std::lround(q[i] * sign / kInvSqrt2 * kQuatMax), -kQuatMax, kQuatMax);
    packed |= (static_cast<uint32_t>(value) & kQuatMask) << shift;
    shift -= kQuatBits;
  }
  return packed;
}

/**
 * @brief Unpacks a quaternion encoded by packQuaternion().
 * @param packed The packed 32-bit word.
 * @param quat The quaternion to populate.
 */
inline void unpackQuaternion(uint32_t packed, geometry_msgs::msg::Quaternion& quat) {
  const int largest = static_cast<int>(packed >> kSelectorShift);
  double comp[4];
  double sum_squares = 0.0;
  int shift = 2 * kQuatBits;
  for (int i = 0; i < 4; ++i) {
    if (i == largest) continue;
    const uint32_t raw = (packed >> shift) & kQuatMask;
    const long value = (raw > static_cast<uint32_t>(kQuatMax))
                           ? static_cast<long>(raw) - (kQuatMask + 1)
                           : static_cast<long>(raw);
    comp[i] = static_cast<double>(value) / kQuatMax * kInvSqrt2;
    sum_squares += comp[i] * comp[i];
    shift -= kQuatBits;
  }
  comp[largest] = std::sqrt(std::max(0.0, 1.0 - sum_squares));
  quat.x = comp[0];
  quat.y = comp[1];
  quat.z = comp[2];
  quat.w = comp[3];
}

/**
 * @brief Encodes an AgentStatus into a Seatrac DAT payload.
 * @param status The status to encode.
 * @param buf Output buffer; must hold at least kStatusPacketLen bytes.
 * @return The number of bytes written (kStatusPacketLen).
 */
inline uint8_t encodeStatus(const coug_interfaces::msg::AgentStatus& status, uint8_t* buf) {
  size_t off = 0;
  writeLE<uint8_t>(buf, off, static_cast<uint8_t>(MsgId::MSG_ORIGIN));

  const auto& pose = status.local_odometry;
  writeLE<int16_t>(buf, off, encodeMeters(pose.position.x));
  writeLE<int16_t>(buf, off, encodeMeters(pose.position.y));
  writeLE<int16_t>(buf, off, encodeMeters(pose.position.z));
  writeLE<uint32_t>(buf, off, packQuaternion(pose.orientation));

  for (int i = 0; i < 6; ++i) {  // 7*i walks the diagonal of the 6x6
    writeLE<uint16_t>(buf, off, floatToHalf(static_cast<float>(status.odometry_covariance[7 * i])));
  }

  writeLE<int16_t>(buf, off, encodeMeters(status.pressure_depth));

  writeLE<uint32_t>(buf, off, packQuaternion(status.imu_orientation));
  return static_cast<uint8_t>(off);
}

/**
 * @brief Decodes a Seatrac DAT payload into an AgentStatus.
 * @param buf The received payload.
 * @param len The payload length.
 * @param status The status to populate (header is left untouched).
 * @return True if the payload is long enough and carries the MSG_ORIGIN
 * discriminator; false otherwise.
 */
inline bool decodeStatus(const uint8_t* buf, uint8_t len,
                         coug_interfaces::msg::AgentStatus& status) {
  if (len < kStatusPacketLen) return false;
  size_t off = 0;
  if (readLE<uint8_t>(buf, off) != static_cast<uint8_t>(MsgId::MSG_ORIGIN)) return false;

  auto& pose = status.local_odometry;
  pose.position.x = decodeMeters(readLE<int16_t>(buf, off));
  pose.position.y = decodeMeters(readLE<int16_t>(buf, off));
  pose.position.z = decodeMeters(readLE<int16_t>(buf, off));
  unpackQuaternion(readLE<uint32_t>(buf, off), pose.orientation);

  status.odometry_covariance.fill(0.0);
  for (int i = 0; i < 6; ++i) {  // 7*i walks the diagonal of the 6x6
    status.odometry_covariance[7 * i] = halfToFloat(readLE<uint16_t>(buf, off));
  }

  status.pressure_depth = decodeMeters(readLE<int16_t>(buf, off));

  unpackQuaternion(readLE<uint32_t>(buf, off), status.imu_orientation);
  return true;
}

}  // namespace coug_comms::utils
