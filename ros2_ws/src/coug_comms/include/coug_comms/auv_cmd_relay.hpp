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
 * @file auv_cmd_relay.hpp
 * @brief ROS 2 node for receiving base station commands on the AUV.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <deque>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_comms/auv_cmd_relay_parameters.hpp"
#include "coug_comms/utils/comms_protocol.hpp"

namespace coug_comms {

/**
 * @class AuvCmdRelayNode
 * @brief ROS 2 node for receiving base station commands on the AUV.
 */
class AuvCmdRelayNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up base station command reception.
   * @param options The node options.
   */
  explicit AuvCmdRelayNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Dispatches incoming commands to the appropriate service.
   * @param msg Incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Calls a Trigger service and logs the outcome.
   * @param client The service client to call.
   * @param cmd The command message ID (for logging).
   */
  void callCommandService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                          utils::MsgId cmd);

  /**
   * @brief Records a command outcome in the rolling history, trimming to the last few.
   * @param command The name of the command that was relayed.
   * @param succeeded Whether the service call succeeded.
   */
  void recordCommandResult(const std::string& command, bool succeeded);

  /**
   * @brief Diagnostic task reporting the last few commands received and their results.
   * @param stat The diagnostic status wrapper.
   */
  void checkCommandStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr surface_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_surface_client_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<auv_cmd_relay_node::ParamListener> param_listener_;
  auv_cmd_relay_node::Params params_;

  // --- State ---
  /**
   * @struct CommandResult
   * @brief A single relayed command and whether its service call succeeded.
   */
  struct CommandResult {
    std::string command;
    bool succeeded;
  };
  std::deque<CommandResult> command_history_;
};

}  // namespace coug_comms
