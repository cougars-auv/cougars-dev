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
 * @file base_status_relay.cpp
 * @brief Implementation of the BaseStatusRelayNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/base_status_relay.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/comms_protocol.hpp"
#include "coug_comms/utils/seatrac_enums.hpp"
#include "coug_comms/utils/status_codec.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::CST_XCVR_RESP_TIMEOUT;
using utils::MSG_REQ;
using utils::MsgId;

BaseStatusRelayNode::BaseStatusRelayNode(const rclcpp::NodeOptions& options)
    : Node("base_status_relay_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Base Status Relay Node...");

  param_listener_ =
      std::make_shared<base_status_relay_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusRelayNode::modemRecCallback, this, std::placeholders::_1));
  modem_cmd_update_sub_ = create_subscription<seatrac_interfaces::msg::ModemCmdUpdate>(
      params_.modem_cmd_update_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&BaseStatusRelayNode::modemCmdUpdateCallback, this, std::placeholders::_1));

  // --- ROS Diagnostics ---
  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_status_relay_node");
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
                                  std::bind(&BaseStatusRelayNode::onTick, this));

  RCLCPP_INFO(get_logger(), "Startup complete! Polling agents for status...");
}

void BaseStatusRelayNode::registerAgent(const std::string& aname, uint8_t beacon_id,
                                        const std::string& diag_prefix) {
  AgentEntry a;
  a.name = aname;
  a.beacon_id = beacon_id;
  a.status_pub = create_publisher<coug_interfaces::msg::AgentStatus>(
      aname + "/" + params_.status_topic, rclcpp::SystemDefaultsQoS());
  a.last_response_time = now();

  if (params_.enable_direct_comms) {
    a.direct_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
        "/" + aname + "/" + params_.status_topic, rclcpp::SystemDefaultsQoS(),
        [this, beacon_id](const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
          directStatusCallback(beacon_id, msg);
        });
  }

  agents_.emplace(beacon_id, std::move(a));
  beacon_order_.push_back(beacon_id);

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Status Poll (" + aname + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", aname.c_str(), beacon_id);
}

void BaseStatusRelayNode::onTick() {
  // Backup timeout: fires if the modem never reports a TIMEOUT (see
  // modemCmdUpdateCallback) and no response arrives in time.
  if (awaiting_response_ && (now() - request_time_).seconds() > params_.response_timeout_sec) {
    failPendingRequest("timed out (no modem report)");
  }
  pollNextIfReady();
}

void BaseStatusRelayNode::pollNextIfReady() {
  if (awaiting_response_ || beacon_order_.empty()) return;
  if (now() < next_poll_allowed_) return;

  AgentEntry& agent = agents_.at(beacon_order_[next_index_]);
  next_index_ = (next_index_ + 1) % beacon_order_.size();

  if (params_.enable_direct_comms && directStatusRelay(agent)) {
    next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
    return;
  }
  if (params_.enable_acoustic_comms) {
    acousticStatusRelay(agent);
    return;
  }
  next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
}

bool BaseStatusRelayNode::directStatusRelay(AgentEntry& agent) {
  // A live direct link relays status from directStatusCallback (push), so this
  // agent does not need to be polled acoustically.
  return agent.direct_sub && agent.direct_sub->get_publisher_count() > 0;
}

void BaseStatusRelayNode::acousticStatusRelay(AgentEntry& agent) {
  seatrac_interfaces::msg::ModemSend send;
  send.msg_id = CID_DAT_SEND;
  send.dest_id = agent.beacon_id;
  send.msg_type = MSG_REQ;  // request a response (the agent's queued status)
  send.packet_len = 1;
  send.packet_data[0] = static_cast<uint8_t>(MsgId::REQ_STATUS);
  modem_send_pub_->publish(send);

  awaiting_response_ = true;
  pending_beacon_ = agent.beacon_id;
  request_time_ = now();
}

void BaseStatusRelayNode::directStatusCallback(
    uint8_t beacon_id, const coug_interfaces::msg::AgentStatus::SharedPtr msg) {
  auto it = agents_.find(beacon_id);
  if (it != agents_.end()) publishStatus(it->second, *msg, "DIRECT");
}

void BaseStatusRelayNode::publishStatus(AgentEntry& agent, coug_interfaces::msg::AgentStatus status,
                                        const std::string& transport) {
  status.header.stamp = now();
  status.header.frame_id = agent.name;
  agent.status_pub->publish(status);

  agent.responses++;
  agent.last_ok = true;
  agent.last_transport = transport;
  agent.last_response_time = now();
}

void BaseStatusRelayNode::modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!awaiting_response_ || !msg->local_flag || msg->src_id != pending_beacon_) return;

  auto it = agents_.find(pending_beacon_);
  if (it == agents_.end()) {
    awaiting_response_ = false;
    return;
  }

  coug_interfaces::msg::AgentStatus status;
  if (!utils::decodeStatus(msg->packet_data.data(), msg->packet_len, status)) {
    // Addressed to us from the polled beacon but not a status response; keep
    // waiting for the real one (or let it time out).
    return;
  }

  publishStatus(it->second, status, "ACOUSTIC");

  awaiting_response_ = false;
  next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
  pollNextIfReady();
}

void BaseStatusRelayNode::modemCmdUpdateCallback(
    const seatrac_interfaces::msg::ModemCmdUpdate::SharedPtr msg) {
  // The modem reports a response timeout for the beacon we are polling; abandon
  // the request now instead of waiting out the backup timer in onTick().
  if (!awaiting_response_ || msg->target_id != pending_beacon_) return;
  if (msg->command_status_code != CST_XCVR_RESP_TIMEOUT) return;

  failPendingRequest("reported a modem response timeout");
  pollNextIfReady();
}

void BaseStatusRelayNode::failPendingRequest(const char* reason) {
  RCLCPP_WARN(get_logger(), "Status request to beacon %d %s.", pending_beacon_, reason);
  recordTimeout(pending_beacon_);
  awaiting_response_ = false;
  next_poll_allowed_ = now() + rclcpp::Duration::from_seconds(params_.poll_period_sec);
}

void BaseStatusRelayNode::recordTimeout(uint8_t beacon_id) {
  auto it = agents_.find(beacon_id);
  if (it == agents_.end()) return;
  it->second.timeouts++;
  it->second.last_ok = false;
}

void BaseStatusRelayNode::checkAgentStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                           uint8_t beacon_id) {
  const AgentEntry& a = agents_.at(beacon_id);

  if (a.responses == 0 && a.timeouts == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first response.");
    return;
  }

  stat.add("Responses", std::to_string(a.responses));
  stat.add("Timeouts", std::to_string(a.timeouts));
  if (a.responses > 0) {
    stat.add("Transport", a.last_transport);
    stat.add("Last response", std::to_string((now() - a.last_response_time).seconds()) + " s ago");
  }

  if (a.last_ok) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Receiving status from " + a.name + ".");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No response from " + a.name + ".");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseStatusRelayNode)
