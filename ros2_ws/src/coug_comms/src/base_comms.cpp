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
 * @file base_comms.cpp
 * @brief Implementation of the BaseCommsNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/base_comms.hpp"

#include <cstring>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"

namespace coug_comms {

using utils::AcousticAck;
using utils::CID_DAT_SEND;
using utils::CmdId;
using utils::MSG_OWAY;

BaseCommsNode::BaseCommsNode(const rclcpp::NodeOptions& options)
    : Node("base_comms_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Base Comms Node...");

  param_listener_ =
      std::make_shared<base_comms_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  // --- ROS Interfaces ---
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      "modem_rec", 10, std::bind(&BaseCommsNode::modemRecCallback, this, std::placeholders::_1));

  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      "modem_send", rclcpp::SystemDefaultsQoS());

  // --- ROS Diagnostics ---
  std::string ns = this->get_namespace();
  std::string clean_ns = (ns == "/") ? "" : ns;
  if (params_.publish_diagnostics) {
    diagnostic_updater_.setHardwareID(clean_ns.empty() ? "base_comms_node"
                                                       : (clean_ns + "/base_comms_node"));
  }

  std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";

  for (const auto& aname : agent_namespaces) {
    int raw_id = this->declare_parameter<int>("beacon_ids." + aname, -1);
    if (raw_id < 0 || raw_id > 15) {
      RCLCPP_ERROR(get_logger(), "Missing or invalid beacon_ids.%s (got %d) — skipping '%s'.",
                   aname.c_str(), raw_id, aname.c_str());
      continue;
    }
    const uint8_t beacon_id = static_cast<uint8_t>(raw_id);

    auto make_srv = [&](const std::string& svc_name, const std::string& label, CmdId cmd) {
      std::string full = "/" + aname + "/" + svc_name;
      return create_service<std_srvs::srv::Trigger>(
          full,
          [this, cmd, beacon_id, label](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            sendCmd(cmd, beacon_id);
            res->success = true;
            res->message = label + " command sent";
          });
    };

    AgentEntry a;
    a.name = aname;
    a.beacon_id = beacon_id;
    a.start_srv = make_srv(params_.start_service, "Start", CmdId::CMD_START);
    a.stop_srv = make_srv(params_.stop_service, "Stop", CmdId::CMD_STOP);
    a.surface_srv = make_srv(params_.surface_service, "Surface", CmdId::CMD_SURFACE);
    a.home_srv = make_srv(params_.home_service, "Home", CmdId::CMD_HOME);

    agents_.emplace(beacon_id, std::move(a));

    if (params_.publish_diagnostics) {
      std::string task = prefix + "Link Status (" + aname + ")";
      diagnostic_updater_.add(task,
                              [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                                checkAgentStatus(stat, beacon_id);
                              });
    }

    RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", aname.c_str(), beacon_id);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for acoustic data.");
}

void BaseCommsNode::modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (msg->packet_len < sizeof(AcousticAck)) return;

  auto it = agents_.find(msg->src_id);
  if (it == agents_.end()) return;

  AgentEntry& a = it->second;
  const auto* ack = reinterpret_cast<const AcousticAck*>(msg->packet_data.data());

  switch (static_cast<CmdId>(ack->msg_id)) {
    case CmdId::ACK_START:
    case CmdId::ACK_STOP:
    case CmdId::ACK_SURFACE:
    case CmdId::ACK_HOME:
      a.last_ack_time = this->get_clock()->now().seconds();
      a.last_ack_success = ack->success;
      RCLCPP_INFO(get_logger(), "ACK 0x%02x from '%s': %s", ack->msg_id, a.name.c_str(),
                  ack->success ? "success" : "failure");
      break;
    default:
      break;
  }
}

void BaseCommsNode::sendCmd(CmdId cmd, uint8_t dest_id) { sendAcousticCmd(cmd, dest_id); }

void BaseCommsNode::sendAcousticCmd(CmdId cmd, uint8_t dest_id) {
  seatrac_interfaces::msg::ModemSend msg;
  msg.msg_id = CID_DAT_SEND;
  msg.dest_id = dest_id;
  msg.msg_type = MSG_OWAY;
  msg.packet_len = 1;
  msg.packet_data[0] = static_cast<uint8_t>(cmd);

  modem_send_pub_->publish(msg);
  RCLCPP_DEBUG(get_logger(), "Sent acoustic cmd 0x%02x to beacon %d", static_cast<uint8_t>(cmd),
               dest_id);
}

void BaseCommsNode::checkAgentStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                     uint8_t beacon_id) {
  const AgentEntry& a = agents_.at(beacon_id);
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ACK received from " + a.name + ".");

  double time_since =
      (a.last_ack_time > 0.0) ? (this->get_clock()->now().seconds() - a.last_ack_time) : -1.0;
  stat.add("Time Since Last (s)", time_since);
  stat.add("Last ACK Success", a.last_ack_success);

  if (a.last_ack_time == 0.0) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                      "Waiting for first ACK from " + a.name + ".");
  } else if (time_since > params_.diagnostic_timeout) {
    stat.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, a.name + " is offline.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseCommsNode)
