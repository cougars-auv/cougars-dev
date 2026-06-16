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
 * @file base_cmd_relay.cpp
 * @brief Implementation of the BaseCmdRelayNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/base_cmd_relay.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::CmdId;
using utils::MSG_OWAY;

BaseCmdRelayNode::BaseCmdRelayNode(const rclcpp::NodeOptions& options)
    : Node("base_cmd_relay_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Base Command Relay Node...");

  param_listener_ =
      std::make_shared<base_cmd_relay_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());

  commands_ = {
      {params_.start_service, params_.direct_start_service, CmdId::CMD_START},
      {params_.stop_service, params_.direct_stop_service, CmdId::CMD_STOP},
      {params_.surface_service, params_.direct_surface_service, CmdId::CMD_SURFACE},
      {params_.home_service, params_.direct_home_service, CmdId::CMD_HOME},
      {params_.emergency_stop_service, params_.direct_emergency_stop_service,
       CmdId::CMD_EMERGENCY_STOP},
      {params_.emergency_surface_service, params_.direct_emergency_surface_service,
       CmdId::CMD_EMERGENCY_SURFACE},
  };

  // --- ROS Diagnostics ---
  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_cmd_relay_node");
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

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for commands...");
}

void BaseCmdRelayNode::registerAgent(const std::string& aname, uint8_t beacon_id,
                                     const std::string& diag_prefix) {
  AgentEntry a;
  a.name = aname;
  a.beacon_id = beacon_id;

  for (const auto& spec : commands_) {
    const CmdId cmd = spec.cmd;
    a.services.push_back(create_service<std_srvs::srv::Trigger>(
        "/" + aname + "/" + spec.relay_service,
        [this, cmd, beacon_id](std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
                               std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<std_srvs::srv::Trigger::Request>) {
          handleCommandRequest(cmd, beacon_id, service, header);
        }));
    a.direct_clients[static_cast<uint8_t>(cmd)] =
        create_client<std_srvs::srv::Trigger>("/" + aname + "/" + spec.direct_service);
  }

  agents_.emplace(beacon_id, std::move(a));

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Command Status (" + aname + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", aname.c_str(), beacon_id);
}

void BaseCmdRelayNode::handleCommandRequest(
    CmdId cmd, uint8_t beacon_id, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> header) {
  AgentEntry& agent = agents_.at(beacon_id);
  const bool direct = !params_.force_acomms && directRelay(cmd, agent, service, header);
  if (!direct) acousticRelay(cmd, beacon_id, service, header);

  agent.last_command = utils::commandName(cmd);
  agent.last_transport = direct ? "DIRECT" : "ACOUSTIC";
}

bool BaseCmdRelayNode::directRelay(CmdId cmd, const AgentEntry& agent,
                                   rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                                   std::shared_ptr<rmw_request_id_t> header) {
  auto client_it = agent.direct_clients.find(static_cast<uint8_t>(cmd));
  if (client_it == agent.direct_clients.end() || !client_it->second->service_is_ready()) {
    return false;
  }

  const std::string label = utils::commandName(cmd);
  client_it->second->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, service, header, label,
       agent_name = agent.name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
        std_srvs::srv::Trigger::Response res;
        res.success = success;
        if (success) {
          res.message = label + " sent";
          service->send_response(*header, res);
          RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
        } else {
          res.message = label + " failed";
          service->send_response(*header, res);
          RCLCPP_WARN(get_logger(), "%s", res.message.c_str());
        }
      });
  return true;
}

void BaseCmdRelayNode::acousticRelay(CmdId cmd, uint8_t beacon_id,
                                     rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                                     std::shared_ptr<rmw_request_id_t> header) {
  seatrac_interfaces::msg::ModemSend msg;
  msg.msg_id = CID_DAT_SEND;
  msg.dest_id = beacon_id;
  msg.msg_type = MSG_OWAY;
  msg.packet_len = 1;
  msg.packet_data[0] = static_cast<uint8_t>(cmd);
  modem_send_pub_->publish(msg);

  std_srvs::srv::Trigger::Response res;
  res.success = true;
  res.message = utils::commandName(cmd) + " queued";
  service->send_response(*header, res);
  RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
}

void BaseCmdRelayNode::checkAgentStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                        uint8_t beacon_id) {
  const AgentEntry& a = agents_.at(beacon_id);

  if (a.last_command.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No commands relayed.");
    return;
  }

  stat.add("Last Command", a.last_command);
  stat.add("Transport", a.last_transport);
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
               "Last relayed " + a.last_command + " to " + a.name + ".");
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseCmdRelayNode)
