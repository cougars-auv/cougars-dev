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
 * @file base_status_relay.hpp
 * @brief ROS 2 node for polling AUV statuses over the acoustic modem.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <cstdint>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <seatrac_interfaces/msg/modem_cmd_update.hpp>
#include <seatrac_interfaces/msg/modem_rec.hpp>
#include <seatrac_interfaces/msg/modem_send.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_status_relay_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

/**
 * @class BaseStatusRelayNode
 * @brief ROS 2 node for polling AUV statuses over the acoustic modem.
 */
class BaseStatusRelayNode : public rclcpp::Node {
 protected:
  /**
   * @struct AgentEntry
   * @brief Per-agent state: identity, status publisher, direct link, and poll statistics.
   */
  struct AgentEntry {
    std::string name;
    uint8_t beacon_id;
    rclcpp::Publisher<coug_interfaces::msg::AgentStatus>::SharedPtr status_pub;
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr direct_sub;
    size_t responses = 0;
    size_t timeouts = 0;
    bool last_ok = false;
    std::string last_transport;
    rclcpp::Time last_response_time;
  };

 public:
  /**
   * @brief Constructs the node and sets up status polling.
   * @param options The node options.
   */
  explicit BaseStatusRelayNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Creates a status publisher and diagnostic task for one agent.
   * @param name The agent's ROS namespace.
   * @param beacon_id The agent's acoustic beacon ID.
   * @param diag_prefix Namespace prefix for diagnostic task labels.
   */
  void registerAgent(const std::string& name, uint8_t beacon_id, const std::string& diag_prefix);

  /**
   * @brief Timer callback that times out stale requests and drives polling.
   */
  void onTick();

  /**
   * @brief Polls the next agent (direct link first, acoustics otherwise) if ready.
   */
  void pollNextIfReady();

  /**
   * @brief Reports whether the agent's direct ROS link is up (its callback then
   * relays status, so it needn't be polled acoustically).
   * @param agent The target agent.
   * @return True if a direct link is available; false to fall back to acoustics.
   */
  bool directStatusRelay(AgentEntry& agent);

  /**
   * @brief Sends an acoustic status request to one agent and marks the channel busy.
   * @param agent The target agent.
   */
  void acousticStatusRelay(AgentEntry& agent);

  /**
   * @brief Republishes a status received over an agent's direct ROS link.
   * @param beacon_id The agent's beacon ID.
   * @param msg The incoming agent status.
   */
  void directStatusCallback(uint8_t beacon_id,
                            const coug_interfaces::msg::AgentStatus::SharedPtr msg);

  /**
   * @brief Republishes an agent's status and records the successful relay.
   * @param agent The agent the status belongs to.
   * @param status The status to republish.
   * @param transport The transport it arrived on ("DIRECT" or "ACOUSTIC").
   */
  void publishStatus(AgentEntry& agent, coug_interfaces::msg::AgentStatus status,
                     const std::string& transport);

  /**
   * @brief Decodes and republishes an acoustic status response from the polled agent.
   * @param msg The incoming modem message.
   */
  void modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg);

  /**
   * @brief Abandons the pending request early if the modem reports a response timeout
   * for the polled beacon. The wall-timer timeout in onTick() remains a backup.
   * @param msg The incoming modem command status update.
   */
  void modemCmdUpdateCallback(const seatrac_interfaces::msg::ModemCmdUpdate::SharedPtr msg);

  /**
   * @brief Records a poll timeout, frees the channel, and schedules the next poll.
   * @param reason Human-readable cause for the warning log.
   */
  void failPendingRequest(const char* reason);

  /**
   * @brief Records a poll timeout for one agent.
   * @param beacon_id The agent's beacon ID.
   */
  void recordTimeout(uint8_t beacon_id);

  /**
   * @brief Diagnostic task reporting one agent's poll responses and timeouts.
   * @param stat The diagnostic status wrapper.
   * @param beacon_id The agent's beacon ID.
   */
  void checkAgentStatus(diagnostic_updater::DiagnosticStatusWrapper& stat, uint8_t beacon_id);

  // --- ROS Interfaces ---
  rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_rec_sub_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemCmdUpdate>::SharedPtr modem_cmd_update_sub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_send_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  diagnostic_updater::Updater diagnostic_updater_;

  // --- Parameters ---
  std::shared_ptr<base_status_relay_node::ParamListener> param_listener_;
  base_status_relay_node::Params params_;

  // --- State ---
  std::vector<uint8_t> beacon_order_;
  std::unordered_map<uint8_t, AgentEntry> agents_;
  size_t next_index_ = 0;
  bool awaiting_response_ = false;
  uint8_t pending_beacon_ = 0;
  rclcpp::Time request_time_;
  rclcpp::Time next_poll_allowed_;
};

}  // namespace coug_comms
