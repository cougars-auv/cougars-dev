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
 * @file auv_receiver.hpp
 * @brief ROS 2 node for receiving base station services on the AUV.
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

#include "coug_comms/auv_receiver_parameters.hpp"
#include "coug_comms/utils/comms_protocol.hpp"

namespace coug_comms {

/**
 * @class AuvReceiverNode
 * @brief ROS 2 node for receiving base station services on the AUV.
 */
class AuvReceiverNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up base station service reception.
   * @param options The node options.
   */
  explicit AuvReceiverNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Dispatches an incoming service request to the appropriate client.
   * @param msg Incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Calls a Trigger service and logs the outcome.
   * @param client The service client to call.
   * @param cmd The service message ID (for logging).
   */
  void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, utils::MsgId cmd);

  /**
   * @brief Records a service outcome in the rolling history, trimming to the last few.
   * @param service The name of the service that was relayed.
   * @param transport The transport used (e.g. "ACOUSTIC").
   * @param succeeded Whether the service call succeeded.
   */
  void recordServiceResult(const std::string& service, const std::string& transport,
                           bool succeeded);

  /**
   * @brief Diagnostic task reporting the last few services received and their results.
   * @param stat The diagnostic status wrapper.
   */
  void checkServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

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
  std::shared_ptr<auv_receiver_node::ParamListener> param_listener_;
  auv_receiver_node::Params params_;

  // --- State ---
  /**
   * @struct ServiceResult
   * @brief A single relayed service and whether its call succeeded.
   */
  struct ServiceResult {
    std::string service;
    std::string transport;
    bool succeeded;
  };
  std::deque<ServiceResult> service_history_;
};

}  // namespace coug_comms
