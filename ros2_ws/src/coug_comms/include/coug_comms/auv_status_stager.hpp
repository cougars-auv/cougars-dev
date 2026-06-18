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
 * @file auv_status_stager.hpp
 * @brief ROS 2 node for staging the AUV's status for acoustic relay.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>

#include "coug_comms/auv_status_stager_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

/**
 * @class AuvStatusStagerNode
 * @brief ROS 2 node for staging the AUV's status for acoustic relay.
 */
class AuvStatusStagerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up status staging.
   * @param options The node options.
   */
  explicit AuvStatusStagerNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Encodes an incoming status and stages it in the modem data queue.
   * @param msg The latest agent status.
   */
  void statusCallback(const coug_interfaces::msg::AgentStatus::SharedPtr msg);

  /**
   * @brief Diagnostic task reporting whether a status was staged recently.
   * @param stat The diagnostic status wrapper.
   */
  void checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<auv_status_stager_node::ParamListener> param_listener_;
  auv_status_stager_node::Params params_;

  // --- State ---
  double last_status_time_{0.0};
};

}  // namespace coug_comms
