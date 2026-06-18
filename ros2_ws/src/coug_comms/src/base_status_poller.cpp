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
 * @file base_status_poller.cpp
 * @brief Implementation of the BaseStatusPollerNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/base_status_poller.hpp"

#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/comms_protocol.hpp"
#include "coug_comms/utils/seatrac_enums.hpp"
#include "coug_comms/utils/status_codec.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::CST_XCVR_RESP_TIMEOUT;
using utils::MSG_REQX;
using utils::MsgId;

BaseStatusPollerNode::BaseStatusPollerNode(const rclcpp::NodeOptions& options)
    : Node("base_status_poller_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Base Status Poller Node...");

  param_listener_ =
      std::make_shared<base_status_poller_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusPollerNode::modemRecCallback, this, std::placeholders::_1));
  modem_cmd_update_sub_ = create_subscription<seatrac_interfaces::msg::ModemCmdUpdate>(
      params_.modem_cmd_update_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusPollerNode::modemCmdUpdateCallback, this, std::placeholders::_1));

  // --- ROS Diagnostics ---
  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_status_poller_node");
    prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
  }

  for (const auto& aname : agent_namespaces) {
    int raw_id = this->declare_parameter<int>("beacon_ids." + aname, -1);
    if (raw_id < 0 || raw_id > 15) {
      RCLCPP_ERROR(get_logger(), "Missing or invalid beacon_ids.%s (got %d) — skipping '%s'.",
                   aname.c_str(), raw_id, aname.c_str());
      continue;
    }
    registerAgent(aname, static_cast<uint8_t>(raw_id), prefix);
  }

  next_poll_allowed_ = now();
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(params_.tick_period_sec),
                                  std::bind(&BaseStatusPollerNode::tickCallback, this));

  RCLCPP_INFO(get_logger(), "Startup complete! Polling agents for status...");
}

void BaseStatusPollerNode::registerAgent(const std::string& aname, uint8_t beacon_id,
                                         const std::string& diag_prefix) {
  AgentEntry a;
  a.name = aname;
  a.beacon_id = beacon_id;
  a.status_pub = create_publisher<coug_interfaces::msg::AgentStatus>(
      "/" + aname + "/" + params_.status_topic, rclcpp::SystemDefaultsQoS());
  a.last_response_time = now();

  if (params_.enable_direct_comms) {
    a.direct_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
        "/" + aname + "/" + params_.direct_status_topic, rclcpp::SystemDefaultsQoS(),
        [this, beacon_id](const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
          auto it = agents_.find(beacon_id);
          if (it == agents_.end()) return;
          it->second.last_direct_heartbeat_sec = now().seconds();
          publishStatus(it->second, *msg, "DIRECT");
        });
  }

  agents_.emplace(beacon_id, std::move(a));
  beacon_order_.push_back(beacon_id);

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Polling Status (" + aname + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentPollStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", aname.c_str(), beacon_id);
}

void BaseStatusPollerNode::tickCallback() {
  if (awaiting_response_ && (now() - request_time_).seconds() > params_.response_timeout_sec) {
    failPendingRequest("missed driver report, node-level response timeout");
  }
  pollNextIfReady();
}

void BaseStatusPollerNode::pollNextIfReady() {
  if (awaiting_response_ || beacon_order_.empty()) return;
  if (now() < next_poll_allowed_) return;

  AgentEntry& agent = agents_.at(beacon_order_[next_index_]);
  next_index_ = (next_index_ + 1) % beacon_order_.size();

  const bool direct_link_up =
      agent.last_direct_heartbeat_sec > 0.0 &&
      now().seconds() - agent.last_direct_heartbeat_sec < params_.direct_timeout_sec;
  if (params_.enable_direct_comms && direct_link_up) {
    scheduleNextPoll();
    return;
  }
  if (params_.enable_acoustic_comms) {
    sendAcousticPoll(agent);
    return;
  }
  scheduleNextPoll();
}

void BaseStatusPollerNode::scheduleNextPoll() {
  next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
}

void BaseStatusPollerNode::sendAcousticPoll(AgentEntry& agent) {
  seatrac_interfaces::msg::ModemSend send;
  send.msg_id = CID_DAT_SEND;
  send.dest_id = agent.beacon_id;
  send.msg_type = MSG_REQX;
  send.packet_len = 1;
  send.packet_data[0] = static_cast<uint8_t>(MsgId::REQ_STATUS);
  modem_send_pub_->publish(send);

  awaiting_response_ = true;
  pending_beacon_ = agent.beacon_id;
  request_time_ = now();
}

void BaseStatusPollerNode::publishStatus(AgentEntry& agent,
                                         coug_interfaces::msg::AgentStatus status,
                                         const std::string& transport) {
  status.header.stamp = now();
  agent.status_pub->publish(status);

  agent.responses++;
  agent.last_transport = transport;
  agent.last_response_time = now();
}

void BaseStatusPollerNode::modemRecCallback(
    const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!awaiting_response_ || !msg->local_flag || msg->src_id != pending_beacon_) return;

  auto it = agents_.find(pending_beacon_);
  if (it == agents_.end()) {
    awaiting_response_ = false;
    return;
  }

  coug_interfaces::msg::AgentStatus status;
  if (!utils::decodeStatus(msg->packet_data.data(), msg->packet_len, status)) {
    return;
  }

  status.includes_range = msg->includes_range;
  if (msg->includes_range) {
    status.range_dist = msg->range_dist / 10.0;
  } else {
    status.range_dist = 0.0;
  }

  status.includes_usbl = msg->includes_usbl;
  if (msg->includes_usbl) {
    status.usbl_azimuth = (msg->usbl_azimuth / 10.0) * M_PI / 180.0;
    status.usbl_elevation = (msg->usbl_elevation / 10.0) * M_PI / 180.0;
  } else {
    status.usbl_azimuth = 0.0;
    status.usbl_elevation = 0.0;
  }

  status.includes_position = msg->includes_position;
  if (msg->includes_position) {
    status.position_depth = msg->position_depth / 10.0;
  } else {
    status.position_depth = 0.0;
  }

  publishStatus(it->second, status, "ACOUSTIC");

  awaiting_response_ = false;
  scheduleNextPoll();
  pollNextIfReady();
}

void BaseStatusPollerNode::modemCmdUpdateCallback(
    const seatrac_interfaces::msg::ModemCmdUpdate::SharedPtr msg) {
  if (!awaiting_response_ || msg->target_id != pending_beacon_) return;
  if (msg->command_status_code != CST_XCVR_RESP_TIMEOUT) return;

  failPendingRequest("driver-level response timeout");
  pollNextIfReady();
}

void BaseStatusPollerNode::failPendingRequest(const char* reason) {
  RCLCPP_WARN(get_logger(), "Beacon %d: %s.", pending_beacon_, reason);
  awaiting_response_ = false;
  scheduleNextPoll();
}

void BaseStatusPollerNode::checkAgentPollStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                uint8_t beacon_id) {
  const AgentEntry& a = agents_.at(beacon_id);

  double time_since = (a.responses > 0) ? (now() - a.last_response_time).seconds() : -1.0;
  if (a.responses > 0) stat.add("Last Transport", a.last_transport);
  stat.add("Time Since Last (s)", time_since);

  double direct_heartbeat_age =
      (a.last_direct_heartbeat_sec > 0.0) ? (now().seconds() - a.last_direct_heartbeat_sec) : -1.0;
  stat.add("Time Since Direct Heartbeat (s)", direct_heartbeat_age);

  if (a.responses == 0 || time_since > params_.diagnostic_timeout_sec) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Agent is offline.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Agent is online (" + a.last_transport + ").");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseStatusPollerNode)
