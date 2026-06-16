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
 * @file auv_command_relay.hpp
 * @brief ROS 2 node for receiving base station commands on the AUV.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "coug_comms/auv_command_relay_parameters.hpp"
#include "coug_comms/utils/acomms_protocol.hpp"

namespace coug_comms {

/**
 * @class AuvCommandRelayNode
 * @brief ROS 2 node for receiving base station commands on the AUV.
 */
class AuvCommandRelayNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up base station command reception.
   * @param options The node options.
   */
  explicit AuvCommandRelayNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Dispatches incoming commands to the appropriate service.
   * @param msg Incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Calls a Trigger service and logs the outcome.
   * @param client The service client to call.
   * @param cmd The command being executed (for logging).
   */
  void callCommandService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                          utils::CmdId cmd);

  /**
   * @brief Diagnostic task reporting the last command received and its service call result.
   * @param stat The diagnostic status wrapper.
   */
  void checkBaseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

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
  std::shared_ptr<auv_command_relay_node::ParamListener> param_listener_;
  auv_command_relay_node::Params params_;

  // --- State ---
  std::string last_command_;
  bool last_call_succeeded_{false};
};

}  // namespace coug_comms
