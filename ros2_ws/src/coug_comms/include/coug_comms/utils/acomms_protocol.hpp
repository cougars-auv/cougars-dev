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
 * @file acomms_protocol.hpp
 * @brief Acoustic command protocol for coug_comms.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <cstdint>

namespace coug_comms::utils {

// First byte of every acoustic command packet.
enum class CmdId : uint8_t {
  CMD_START = 0x10,
  CMD_STOP = 0x11,
  CMD_SURFACE = 0x12,
  CMD_HOME = 0x13,
  ACK_START = 0x20,
  ACK_STOP = 0x21,
  ACK_SURFACE = 0x22,
  ACK_HOME = 0x23,
};

// Sent from AUV back to base after executing a command.
struct __attribute__((packed)) AcousticAck {
  uint8_t msg_id;
  uint8_t success;
};

}  // namespace coug_comms::utils
