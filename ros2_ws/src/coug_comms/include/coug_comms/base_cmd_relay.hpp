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
 * @file base_cmd_relay.hpp
 * @brief ROS 2 node for relaying base station commands to AUVs.
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

#include "coug_comms/base_cmd_relay_parameters.hpp"
#include "coug_comms/utils/comms_protocol.hpp"

namespace coug_comms {

/**
 * @class BaseCmdRelayNode
 * @brief ROS 2 node for relaying base station commands to AUVs.
 */
class BaseCmdRelayNode : public rclcpp::Node {
 protected:
  /**
   * @struct CommandSpec
   * @brief Service names and message ID for one relayable command type.
   */
  struct CommandSpec {
    std::string relay_service;
    std::string direct_service;
    utils::MsgId cmd;
  };

  /**
   * @struct CommandResult
   * @brief A single relayed command, the transport used, and whether it succeeded.
   */
  struct CommandResult {
    std::string command;
    std::string transport;
    bool succeeded;
  };

  /**
   * @struct AgentEntry
   * @brief Per-agent state: identity, hosted services, direct clients, and command history.
   */
  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::unordered_map<uint8_t, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> direct_clients;
    std::deque<CommandResult> command_history;
  };

 public:
  /**
   * @brief Constructs the node and sets up command relaying to AUVs.
   * @param options The node options.
   */
  explicit BaseCmdRelayNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Routes the command to the direct link, falling back to acoustics.
   * @param cmd The message ID to send.
   * @param beacon_id The target beacon ID.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   */
  void handleCommandRequest(utils::MsgId cmd, uint8_t beacon_id,
                            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                            std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Sends the command over the agent's direct ROS link, responding to the Trigger
   * asynchronously.
   * @param cmd The message ID to send.
   * @param agent The target agent.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   * @return True if a direct link was ready and the request was sent; false to fall back to
   * acoustics.
   */
  bool directCommandRelay(utils::MsgId cmd, const AgentEntry& agent,
                          rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                          std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Sends the command over the seatrac modem (one-way) and responds that it was queued.
   * @param cmd The message ID to send.
   * @param beacon_id The target beacon ID.
   * @param service The service the request arrived on.
   * @param header The request header to respond to.
   */
  void acousticCommandRelay(utils::MsgId cmd, uint8_t beacon_id,
                            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service,
                            std::shared_ptr<rmw_request_id_t> header);

  /**
   * @brief Creates command services, direct clients, and a diagnostic task for one agent.
   * @param name The agent's ROS namespace.
   * @param beacon_id The agent's acoustic beacon ID.
   * @param diag_prefix Namespace prefix for diagnostic task labels.
   */
  void registerAgent(const std::string& name, uint8_t beacon_id, const std::string& diag_prefix);

  /**
   * @brief Records a command outcome in an agent's rolling history, trimming to the last few.
   * @param beacon_id The agent's beacon ID.
   * @param command The name of the command that was relayed.
   * @param transport The transport used to relay the command.
   * @param succeeded Whether the relay succeeded.
   */
  void recordCommandResult(uint8_t beacon_id, const std::string& command,
                           const std::string& transport, bool succeeded);

  /**
   * @brief Diagnostic task reporting the last few commands relayed to one agent and their results.
   * @param stat The diagnostic status wrapper.
   * @param beacon_id The agent's beacon ID.
   */
  void checkCommandStatus(diagnostic_updater::DiagnosticStatusWrapper& stat, uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<base_cmd_relay_node::ParamListener> param_listener_;
  base_cmd_relay_node::Params params_;

  // --- State ---
  std::vector<CommandSpec> commands_;
  std::unordered_map<uint8_t, AgentEntry> agents_;
};

}  // namespace coug_comms
