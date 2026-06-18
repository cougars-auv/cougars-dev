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
 * @file base_dispatcher.cpp
 * @brief Implementation of the BaseDispatcherNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/base_dispatcher.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "coug_comms/utils/seatrac_enums.hpp"

namespace coug_comms {

using utils::CID_DAT_SEND;
using utils::MSG_OWAY;
using utils::MsgId;

BaseDispatcherNode::BaseDispatcherNode(const rclcpp::NodeOptions& options)
    : Node("base_dispatcher_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting Base Dispatcher Node...");

  param_listener_ =
      std::make_shared<base_dispatcher_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  this->declare_parameter<std::vector<std::string>>("agent_namespaces", std::vector<std::string>{});
  const auto agent_namespaces = this->get_parameter("agent_namespaces").as_string_array();

  // --- ROS Interfaces ---
  modem_send_pub_ = create_publisher<seatrac_interfaces::msg::ModemSend>(
      params_.modem_send_topic, rclcpp::SystemDefaultsQoS());

  services_ = {
      {params_.start_service, params_.direct_start_service, MsgId::SRV_START},
      {params_.stop_service, params_.direct_stop_service, MsgId::SRV_STOP},
      {params_.surface_service, params_.direct_surface_service, MsgId::SRV_SURFACE},
      {params_.home_service, params_.direct_home_service, MsgId::SRV_HOME},
      {params_.emergency_stop_service, params_.direct_emergency_stop_service,
       MsgId::SRV_EMERGENCY_STOP},
      {params_.emergency_surface_service, params_.direct_emergency_surface_service,
       MsgId::SRV_EMERGENCY_SURFACE},
  };

  // --- ROS Diagnostics ---
  std::string prefix;
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/base_dispatcher_node");
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

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for services...");
}

void BaseDispatcherNode::registerAgent(const std::string& aname, uint8_t beacon_id,
                                       const std::string& diag_prefix) {
  AgentEntry a;
  a.name = aname;
  a.beacon_id = beacon_id;

  for (const auto& spec : services_) {
    const MsgId cmd = spec.cmd;
    a.services.push_back(create_service<std_srvs::srv::Trigger>(
        "/" + aname + "/" + spec.relay_service,
        [this, cmd, beacon_id](std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
                               std::shared_ptr<rmw_request_id_t> header,
                               std::shared_ptr<std_srvs::srv::Trigger::Request>) {
          handleServiceRequest(cmd, beacon_id, service, header);
        }));
    a.direct_clients[static_cast<uint8_t>(cmd)] =
        create_client<std_srvs::srv::Trigger>("/" + aname + "/" + spec.direct_service);
  }

  if (params_.enable_direct_comms) {
    a.direct_heartbeat_sub = create_subscription<coug_interfaces::msg::AgentStatus>(
        "/" + aname + "/" + params_.direct_status_topic, rclcpp::SystemDefaultsQoS(),
        [this, beacon_id](const coug_interfaces::msg::AgentStatus::SharedPtr) {
          auto it = agents_.find(beacon_id);
          if (it != agents_.end()) it->second.last_direct_heartbeat_sec = now().seconds();
        });
  }

  agents_.emplace(beacon_id, std::move(a));

  if (params_.publish_diagnostics) {
    diagnostic_updater_.add(diag_prefix + "Service Status (" + aname + ")",
                            [this, beacon_id](diagnostic_updater::DiagnosticStatusWrapper& stat) {
                              checkAgentServiceStatus(stat, beacon_id);
                            });
  }

  RCLCPP_INFO(get_logger(), "Registered agent '%s' (beacon %d).", aname.c_str(), beacon_id);
}

void BaseDispatcherNode::handleServiceRequest(
    MsgId cmd, uint8_t beacon_id, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> header) {
  AgentEntry& agent = agents_.at(beacon_id);
  const std::string name = utils::messageType(cmd);

  if (params_.enable_direct_comms) {
    if (directServiceDispatch(cmd, agent, service, header)) {
      return;
    }
  }

  if (params_.enable_acoustic_comms) {
    acousticServiceDispatch(cmd, beacon_id, service, header);
    recordServiceResult(beacon_id, name, "ACOUSTIC", true);
    return;
  }

  std_srvs::srv::Trigger::Response res;
  res.success = false;
  res.message = name + " failed: comms disabled";
  service->send_response(*header, res);
  RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
  recordServiceResult(beacon_id, name, "NONE", false);
}

bool BaseDispatcherNode::directServiceDispatch(
    MsgId cmd, const AgentEntry& agent, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
    std::shared_ptr<rmw_request_id_t> header) {
  const bool direct_link_up =
      agent.last_direct_heartbeat_sec > 0.0 &&
      now().seconds() - agent.last_direct_heartbeat_sec < params_.direct_timeout_sec;
  auto client_it = agent.direct_clients.find(static_cast<uint8_t>(cmd));
  if (!direct_link_up || client_it == agent.direct_clients.end() ||
      !client_it->second->service_is_ready()) {
    return false;
  }

  const std::string label = utils::messageType(cmd);
  const uint8_t beacon_id = agent.beacon_id;
  client_it->second->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, service, header, label,
       beacon_id](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
        std_srvs::srv::Trigger::Response res;
        res.success = success;
        if (success) {
          res.message = label + " succeeded";
          service->send_response(*header, res);
          RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
        } else {
          res.message = label + " failed";
          service->send_response(*header, res);
          RCLCPP_WARN(get_logger(), "%s", res.message.c_str());
        }
        recordServiceResult(beacon_id, label, "DIRECT", success);
      });
  return true;
}

void BaseDispatcherNode::acousticServiceDispatch(
    MsgId cmd, uint8_t beacon_id, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
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
  res.message = utils::messageType(cmd) + " queued";
  service->send_response(*header, res);
  RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
}

void BaseDispatcherNode::recordServiceResult(uint8_t beacon_id, const std::string& service,
                                             const std::string& transport, bool succeeded) {
  AgentEntry& a = agents_.at(beacon_id);
  a.service_history.push_back({service, transport, succeeded});
  if (a.service_history.size() > static_cast<size_t>(params_.service_history_size)) {
    a.service_history.pop_front();
  }
}

void BaseDispatcherNode::checkAgentServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                 uint8_t beacon_id) {
  const AgentEntry& a = agents_.at(beacon_id);

  double direct_heartbeat_age =
      (a.last_direct_heartbeat_sec > 0.0) ? (now().seconds() - a.last_direct_heartbeat_sec) : -1.0;
  stat.add("Time Since Direct Heartbeat (s)", direct_heartbeat_age);

  if (a.service_history.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first service.");
    return;
  }

  size_t index = 1;
  for (auto it = a.service_history.rbegin(); it != a.service_history.rend(); ++it) {
    stat.add(std::to_string(index++), it->service + " (" + it->transport + ")" +
                                          (it->succeeded ? ": succeeded" : ": failed"));
  }

  const ServiceResult& latest = a.service_history.back();
  if (latest.succeeded) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, latest.service + " succeeded.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, latest.service + " failed.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::BaseDispatcherNode)
