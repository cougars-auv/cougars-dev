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
 * @file comms_protocol.hpp
 * @brief Shared messaging protocol for radio and acoustic comms.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <cstdint>
#include <string>

namespace coug_comms::utils {

enum class MsgId : uint8_t {
  CMD_START = 0x10,
  CMD_STOP = 0x11,
  CMD_SURFACE = 0x12,
  CMD_HOME = 0x13,
  CMD_EMERGENCY_STOP = 0x14,
  CMD_EMERGENCY_SURFACE = 0x15,
  REQ_STATUS = 0x20,
  MSG_KEY_CONTROL = 0x30,
  MSG_ORIGIN = 0x31,
  MSG_WAYPOINTS = 0x32,
};

/**
 * @brief Returns the human-readable name of a message type.
 * @param msg The message type to name.
 * @return The message name, or "MSG_UNKNOWN" if unrecognized.
 */
inline std::string messageType(MsgId msg) {
  switch (msg) {
    case MsgId::CMD_START:
      return "CMD_START";
    case MsgId::CMD_STOP:
      return "CMD_STOP";
    case MsgId::CMD_SURFACE:
      return "CMD_SURFACE";
    case MsgId::CMD_HOME:
      return "CMD_HOME";
    case MsgId::CMD_EMERGENCY_STOP:
      return "CMD_EMERGENCY_STOP";
    case MsgId::CMD_EMERGENCY_SURFACE:
      return "CMD_EMERGENCY_SURFACE";
    case MsgId::REQ_STATUS:
      return "REQ_STATUS";
    case MsgId::MSG_KEY_CONTROL:
      return "MSG_KEY_CONTROL";
    case MsgId::MSG_ORIGIN:
      return "MSG_ORIGIN";
    case MsgId::MSG_WAYPOINTS:
      return "MSG_WAYPOINTS";
  }
  return "MSG_UNKNOWN";
}

}  // namespace coug_comms::utils
