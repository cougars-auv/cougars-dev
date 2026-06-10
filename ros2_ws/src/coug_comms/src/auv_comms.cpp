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
 * @file auv_comms.cpp
 * @brief Implementation of the AuvCommsNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_comms.hpp"

#include <cstring>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"

namespace coug_comms {

using utils::AcousticAck;
using utils::CID_DAT_SEND;
using utils::CmdId;
using utils::MSG_OWAY;
using utils::Transport;

AuvCommsNode::AuvCommsNode(const rclcpp::NodeOptions& options)
    : Node("auv_comms_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting AUV Comms Node...");

  param_listener_ =
      std::make_shared<auv_comms_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      "modem_rec", 10, std::bind(&AuvCommsNode::modemRecCallback, this, std::placeholders::_1));

  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      "modem_send", rclcpp::SystemDefaultsQoS());

  start_client_ = create_client<std_srvs::srv::Trigger>(params_.start_service);
  stop_client_ = create_client<std_srvs::srv::Trigger>(params_.stop_service);
  surface_client_ = create_client<std_srvs::srv::Trigger>(params_.surface_service);
  home_client_ = create_client<std_srvs::srv::Trigger>(params_.home_service);

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/auv_comms_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    std::string cmd_task = prefix + "Command Status";
    diagnostic_updater_.add(cmd_task, this, &AuvCommsNode::checkAuvCommsStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for commands.");
}

void AuvCommsNode::modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!msg->local_flag || msg->packet_len < 1) return;

  const auto id = static_cast<CmdId>(msg->packet_data[0]);
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  CmdId ack_id;
  switch (id) {
    case CmdId::CMD_START:
      client = start_client_;
      ack_id = CmdId::ACK_START;
      break;
    case CmdId::CMD_STOP:
      client = stop_client_;
      ack_id = CmdId::ACK_STOP;
      break;
    case CmdId::CMD_SURFACE:
      client = surface_client_;
      ack_id = CmdId::ACK_SURFACE;
      break;
    case CmdId::CMD_HOME:
      client = home_client_;
      ack_id = CmdId::ACK_HOME;
      break;
    default:
      return;
  }

  RCLCPP_INFO(get_logger(), "Received cmd 0x%02x from beacon %d", static_cast<uint8_t>(id),
              msg->src_id);
  last_cmd_time_ = this->get_clock()->now().seconds();
  callService(client, ack_id, msg->src_id, Transport::ACOUSTIC);
}

void AuvCommsNode::callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                               CmdId ack_id, uint8_t src_id, Transport transport) {
  if (!client->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready for ack 0x%02x — sending NACK",
                static_cast<uint8_t>(ack_id));
    sendAck(ack_id, false, src_id, transport);
    return;
  }
  auto responded = std::make_shared<bool>(false);
  auto timer_holder = std::make_shared<rclcpp::TimerBase::SharedPtr>();
  *timer_holder = create_wall_timer(
      std::chrono::duration<double>(params_.service_timeout),
      [this, timer_holder, responded, ack_id, src_id, transport]() {
        if (*timer_holder) {
          (*timer_holder)->cancel();
          timer_holder->reset();
        }
        if (!*responded) {
          *responded = true;
          RCLCPP_WARN(get_logger(), "Service timed out for ack 0x%02x — sending NACK",
                      static_cast<uint8_t>(ack_id));
          sendAck(ack_id, false, src_id, transport);
        }
      });

  client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, timer_holder, responded, ack_id, src_id,
       transport](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (*timer_holder) {
          (*timer_holder)->cancel();
          timer_holder->reset();
        }
        if (*responded) {
          return;
        }
        *responded = true;
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
        RCLCPP_INFO(get_logger(), "Service ack 0x%02x: %s", static_cast<uint8_t>(ack_id),
                    success ? "success" : "failure");
        sendAck(ack_id, success, src_id, transport);
      });
}

void AuvCommsNode::sendAck(CmdId ack_id, bool success, uint8_t dest_id, Transport transport) {
  switch (transport) {
    case Transport::ACOUSTIC:
      sendAcousticAck(ack_id, success, dest_id);
      break;
    // case Transport::RADIO: sendRadioAck(ack_id, success, dest_id); break;
    // case Transport::WIFI:  sendWifiAck(ack_id, success, dest_id);  break;
    default:
      break;
  }
}

void AuvCommsNode::sendAcousticAck(CmdId ack_id, bool success, uint8_t dest_id) {
  AcousticAck ack{static_cast<uint8_t>(ack_id), static_cast<uint8_t>(success)};

  seatrac_interfaces::msg::ModemSend msg;
  msg.msg_id = CID_DAT_SEND;
  msg.dest_id = dest_id;
  msg.msg_type = MSG_OWAY;
  msg.packet_len = sizeof(AcousticAck);
  std::memcpy(msg.packet_data.data(), &ack, sizeof(AcousticAck));

  modem_send_pub_->publish(msg);
}

void AuvCommsNode::checkAuvCommsStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Command received.");

  double time_since =
      (last_cmd_time_ > 0.0) ? (this->get_clock()->now().seconds() - last_cmd_time_) : -1.0;
  stat.add("Time Since Last (s)", time_since);

  if (last_cmd_time_ == 0.0) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first command.");
  } else if (time_since > params_.diagnostic_timeout) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Base station is offline.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvCommsNode)
