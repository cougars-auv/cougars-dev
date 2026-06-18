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
 * @file base_status_extractor.hpp
 * @brief ROS 2 node for extracting AgentStatus into standard topics.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "coug_comms/base_status_extractor_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

/**
 * @class BaseStatusExtractorNode
 * @brief ROS 2 node for extracting AgentStatus into standard topics.
 */
class BaseStatusExtractorNode : public rclcpp::Node {
 protected:
  /**
   * @struct AgentEntry
   * @brief Per-agent subscriptions and publishers.
   */
  struct AgentEntry {
    std::string name;
    rclcpp::Subscription<coug_interfaces::msg::AgentStatus>::SharedPtr status_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  };

 public:
  /**
   * @brief Constructs the node and sets up extraction subscriptions/publishers.
   * @param options The node options.
   */
  explicit BaseStatusExtractorNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Creates status subscriptions and extraction publishers for one agent.
   * @param aname The agent's ROS namespace.
   */
  void registerAgent(const std::string& aname);

  /**
   * @brief Callback for incoming AgentStatus messages.
   * @param aname The namespace of the agent.
   * @param msg The received status message.
   */
  void statusCallback(const std::string& aname,
                      const coug_interfaces::msg::AgentStatus::SharedPtr msg);

  // --- Parameters ---
  std::shared_ptr<base_status_extractor_node::ParamListener> param_listener_;
  base_status_extractor_node::Params params_;

  // --- State ---
  std::unordered_map<std::string, AgentEntry> agents_;
};

}  // namespace coug_comms
