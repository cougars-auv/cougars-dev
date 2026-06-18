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
 * @file auv_status_stager.cpp
 * @brief Implementation of the AuvStatusStagerNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_status_stager.hpp"

#include <array>
#include <cstdint>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"
#include "coug_comms/utils/status_codec.hpp"

namespace coug_comms {

using utils::BEACON_ALL;
using utils::CID_DAT_QUEUE_SET;

AuvStatusStagerNode::AuvStatusStagerNode(const rclcpp::NodeOptions& options)
    : Node("auv_status_stager_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting AUV Status Stager Node...");

  param_listener_ =
      std::make_shared<auv_status_stager_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());
  status_sub_ = create_subscription<coug_interfaces::msg::AgentStatus>(
      params_.status_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusStagerNode::statusCallback, this, std::placeholders::_1));

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/auv_status_stager_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    diagnostic_updater_.add(prefix + "Staging Status", this, &AuvStatusStagerNode::checkStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for status updates...");
}

void AuvStatusStagerNode::statusCallback(const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  last_status_time_ = this->get_clock()->now().seconds();

  seatrac_interfaces::msg::ModemSend send;
  send.msg_id = CID_DAT_QUEUE_SET;
  send.dest_id = BEACON_ALL;
  std::array<uint8_t, 30> buffer{};
  send.packet_len = utils::encodeStatus(*msg, buffer.data());
  std::copy(buffer.begin(), buffer.end(), send.packet_data.begin());
  modem_send_pub_->publish(send);
}

void AuvStatusStagerNode::checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  double time_since =
      (last_status_time_ > 0.0) ? (this->get_clock()->now().seconds() - last_status_time_) : -1.0;
  stat.add("Time Since Last (s)", time_since);

  if (time_since > params_.diagnostic_timeout_sec || last_status_time_ == 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No status to relay.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Status staged successfully.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvStatusStagerNode)
