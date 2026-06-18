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
 * @file base_dispatcher.hpp
 * @brief ROS 2 node for relaying base station services to AUVs.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <deque>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_dispatcher_parameters.hpp"
#include "coug_comms/utils/comms_protocol.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

/**
 * @class BaseDispatcherNode
 * @brief ROS 2 node for relaying base station services to AUVs.
 */
class BaseDispatcherNode : public rclcpp::Node {
 protected:
  /**
   * @struct ServiceSpec
   * @brief Service names and message ID for one relayable service type.
   */
  struct ServiceSpec {
    std::string relay_service;
    std::string direct_service;
    utils::MsgId cmd;
  };

  /**
   * @struct ServiceResult
   * @brief A single relayed service, the transport used, and whether it succeeded.
   */
  struct ServiceResult {
    std::string service;
    std::string transport;
    bool succeeded;
  };

  /**
   * @struct AgentEntry
   * @brief Per-agent state: identity, services, direct clients, service history, and heartbeat.
   */
  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::unordered_map<uint8_t, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> direct_clients;
    std::deque<ServiceResult> service_history;
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr direct_heartbeat_sub;
    double last_direct_heartbeat_sec = 0.0;
  };

 public:
  /**
   * @brief Constructs the node and sets up service relaying to AUVs.
   * @param options The node options.
   */
  explicit BaseDispatcherNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Routes the service to the direct link, falling back to acoustics.
   * @param cmd The message ID to send.
   * @param beacon_id The target beacon ID.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   */
  void handleServiceRequest(utils::MsgId cmd, uint8_t beacon_id,
                            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                            std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Sends the service over the agent's direct ROS link, responding asynchronously.
   * @param cmd The message ID to send.
   * @param agent The target agent.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   * @return True if the direct link was live and the request was sent; false to fall back.
   */
  bool directServiceDispatch(utils::MsgId cmd, const AgentEntry& agent,
                             rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                             std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Sends the service over the seatrac modem (one-way) and responds that it was queued.
   * @param cmd The message ID to send.
   * @param beacon_id The target beacon ID.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   */
  void acousticServiceDispatch(utils::MsgId cmd, uint8_t beacon_id,
                               rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                               std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Creates service servers, direct clients, a heartbeat subscription, and a diagnostic
   * task.
   * @param name The agent's ROS namespace.
   * @param beacon_id The agent's acoustic beacon ID.
   * @param diag_prefix Namespace prefix for diagnostic task labels.
   */
  void registerAgent(const std::string& name, uint8_t beacon_id, const std::string& diag_prefix);

  /**
   * @brief Records a service outcome in an agent's rolling history, trimming to the last few.
   * @param beacon_id The agent's beacon ID.
   * @param service The name of the service that was relayed.
   * @param transport The transport used to relay the service.
   * @param succeeded Whether the relay succeeded.
   */
  void recordServiceResult(uint8_t beacon_id, const std::string& service,
                           const std::string& transport, bool succeeded);

  /**
   * @brief Diagnostic task reporting an agent's heartbeat and the last few services relayed to it.
   * @param stat The diagnostic status wrapper.
   * @param beacon_id The agent's beacon ID.
   */
  void checkAgentServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat,
                               uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<base_dispatcher_node::ParamListener> param_listener_;
  base_dispatcher_node::Params params_;

  // --- State ---
  std::vector<ServiceSpec> services_;
  std::unordered_map<uint8_t, AgentEntry> agents_;
};

}  // namespace coug_comms
