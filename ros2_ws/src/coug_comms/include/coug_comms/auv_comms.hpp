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
 * @file auv_comms.hpp
 * @brief ROS 2 node for AUV-side modem comms.
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

#include "coug_comms/auv_comms_parameters.hpp"
#include "coug_comms/utils/acomms_protocol.hpp"
#include "coug_comms/utils/comms_types.hpp"

namespace coug_comms {

/**
 * @class AuvCommsNode
 * @brief ROS 2 node for AUV-side modem comms.
 */
class AuvCommsNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up all ROS interfaces.
   * @param options The node options.
   */
  explicit AuvCommsNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Dispatches incoming commands to the appropriate service.
   * @param msg Incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Calls a Trigger service and sends an ACK on completion.
   * @param client The service client to call.
   * @param ack_id The ACK message ID to send on completion.
   * @param src_id The beacon ID to send the ACK back to.
   * @param transport The transport the command arrived on.
   */
  void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, utils::CmdId ack_id,
                   uint8_t src_id, utils::Transport transport);

  /**
   * @brief Dispatches an ACK on the transport the command arrived on.
   * @param ack_id The ACK message ID (one of ACK_*).
   * @param success Whether the service call succeeded.
   * @param dest_id The destination ID to send the ACK to.
   * @param transport The transport to reply on.
   */
  void sendAck(utils::CmdId ack_id, bool success, uint8_t dest_id, utils::Transport transport);

  /**
   * @brief Packs and publishes an AcousticAck via the seatrac modem.
   * @param ack_id The ACK message ID (one of ACK_*).
   * @param success Whether the service call succeeded.
   * @param dest_id The beacon ID to send the ACK to.
   */
  void sendAcousticAck(utils::CmdId ack_id, bool success, uint8_t dest_id);

  /**
   * @brief Diagnostic task reporting the status of the command link.
   * @param stat The diagnostic status wrapper.
   */
  void checkAuvCommsStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr surface_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr home_client_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<auv_comms_node::ParamListener> param_listener_;
  auv_comms_node::Params params_;

  // --- State ---
  double last_cmd_time_{0.0};
};

}  // namespace coug_comms
