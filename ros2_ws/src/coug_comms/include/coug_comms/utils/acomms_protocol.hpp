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
 * @brief Shared acoustic command protocol for coug_comms.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <cstdint>
#include <string>

namespace coug_comms::utils {

enum class CmdId : uint8_t {
  CMD_START = 0x10,
  CMD_STOP = 0x11,
  CMD_SURFACE = 0x12,
  CMD_HOME = 0x13,
  CMD_EMERGENCY_STOP = 0x20,
  CMD_EMERGENCY_SURFACE = 0x21,
};

/**
 * @brief Returns the human-readable name of a command.
 * @param cmd The command to name.
 * @return The command name, or "CMD_UNKNOWN" if unrecognized.
 */
inline std::string commandName(CmdId cmd) {
  switch (cmd) {
    case CmdId::CMD_START:
      return "CMD_START";
    case CmdId::CMD_STOP:
      return "CMD_STOP";
    case CmdId::CMD_SURFACE:
      return "CMD_SURFACE";
    case CmdId::CMD_HOME:
      return "CMD_HOME";
    case CmdId::CMD_EMERGENCY_STOP:
      return "CMD_EMERGENCY_STOP";
    case CmdId::CMD_EMERGENCY_SURFACE:
      return "CMD_EMERGENCY_SURFACE";
  }
  return "CMD_UNKNOWN";
}

}  // namespace coug_comms::utils
