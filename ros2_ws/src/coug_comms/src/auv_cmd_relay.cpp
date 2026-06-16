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
 * @file auv_cmd_relay.cpp
 * @brief Implementation of the AuvCmdRelayNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_cmd_relay.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_comms {

using utils::CmdId;

AuvCmdRelayNode::AuvCmdRelayNode(const rclcpp::NodeOptions& options)
    : Node("auv_cmd_relay_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting AUV Command Relay Node...");

  param_listener_ =
      std::make_shared<auv_cmd_relay_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvCmdRelayNode::modemRecCallback, this, std::placeholders::_1));

  start_client_ = create_client<std_srvs::srv::Trigger>(params_.start_service);
  stop_client_ = create_client<std_srvs::srv::Trigger>(params_.stop_service);
  surface_client_ = create_client<std_srvs::srv::Trigger>(params_.surface_service);
  home_client_ = create_client<std_srvs::srv::Trigger>(params_.home_service);
  emergency_stop_client_ = create_client<std_srvs::srv::Trigger>(params_.emergency_stop_service);
  emergency_surface_client_ =
      create_client<std_srvs::srv::Trigger>(params_.emergency_surface_service);

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/auv_cmd_relay_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    std::string cmd_task = prefix + "Command Status";
    diagnostic_updater_.add(cmd_task, this, &AuvCmdRelayNode::checkBaseStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for acoustic commands...");
}

void AuvCmdRelayNode::modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!msg->local_flag || msg->packet_len < 1) return;

  const auto id = static_cast<CmdId>(msg->packet_data[0]);
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  switch (id) {
    case CmdId::CMD_START:
      client = start_client_;
      break;
    case CmdId::CMD_STOP:
      client = stop_client_;
      break;
    case CmdId::CMD_SURFACE:
      client = surface_client_;
      break;
    case CmdId::CMD_HOME:
      client = home_client_;
      break;
    case CmdId::CMD_EMERGENCY_STOP:
      client = emergency_stop_client_;
      break;
    case CmdId::CMD_EMERGENCY_SURFACE:
      client = emergency_surface_client_;
      break;
    default:
      return;
  }

  RCLCPP_INFO(get_logger(), "Received %s from beacon %d", utils::commandName(id).c_str(),
              msg->src_id);
  callCommandService(client, id);
}

void AuvCmdRelayNode::callCommandService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                                         CmdId cmd) {
  last_command_ = utils::commandName(cmd);
  if (!client->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready for %s", last_command_.c_str());
    last_call_succeeded_ = false;
    return;
  }
  client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, cmd](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
        last_call_succeeded_ = success;
        RCLCPP_INFO(get_logger(), "Service for %s: %s", utils::commandName(cmd).c_str(),
                    success ? "success" : "failure");
      });
}

void AuvCmdRelayNode::checkBaseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (last_command_.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first command.");
    return;
  }

  stat.add("Last Command", last_command_);
  stat.add("Service Call Succeeded", last_call_succeeded_);
  if (last_call_succeeded_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, last_command_ + " succeeded.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, last_command_ + " failed.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvCmdRelayNode)
