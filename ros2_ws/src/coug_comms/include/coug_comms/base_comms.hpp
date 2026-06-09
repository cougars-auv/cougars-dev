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
 * @file base_comms.hpp
 * @brief ROS 2 node for base station modem comms.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>

#include "coug_comms/base_comms_parameters.hpp"
#include "coug_comms/utils/acomms_protocol.hpp"
#include "coug_comms/utils/comms_types.hpp"

namespace coug_comms {

/**
 * @class BaseCommsNode
 * @brief ROS 2 node for base station modem comms.
 */
class BaseCommsNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up all ROS interfaces.
   * @param options The node options.
   */
  explicit BaseCommsNode(const rclcpp::NodeOptions& options);

 protected:
  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    double last_ack_time{0.0};
    bool last_ack_success{false};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr surface_srv;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv;
  };

  /**
   * @brief Logs ACK messages received from any configured agent.
   * @param msg Incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Dispatches a command to an agent using the best available transport.
   * @param cmd The command to send.
   * @param dest_id The target beacon ID.
   */
  void sendCmd(utils::CmdId cmd, uint8_t dest_id);

  /**
   * @brief Packs and publishes a single-byte command packet via the seatrac modem.
   * @param cmd The command to send.
   * @param dest_id The target beacon ID.
   */
  void sendAcousticCmd(utils::CmdId cmd, uint8_t dest_id);

  /**
   * @brief Diagnostic task reporting the link status for one agent.
   * @param stat The diagnostic status wrapper.
   * @param beacon_id The agent's beacon ID (key into agents_).
   */
  void checkAgentStatus(diagnostic_updater::DiagnosticStatusWrapper& stat, uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<base_comms_node::ParamListener> param_listener_;
  base_comms_node::Params params_;

  // --- State ---
  std::unordered_map<uint8_t, AgentEntry> agents_;
};

}  // namespace coug_comms
