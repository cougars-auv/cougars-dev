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
 * @file auv_status_bundler.hpp
 * @brief ROS 2 node for bundling sensor topics into AgentStatus.
 * @author Nelson Durrant
 * @date June 2026
 */

#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "coug_comms/auv_status_bundler_parameters.hpp"
#include "coug_interfaces/msg/agent_status.hpp"

namespace coug_comms {

/**
 * @class AuvStatusBundlerNode
 * @brief ROS 2 node for bundling sensor topics into AgentStatus.
 */
class AuvStatusBundlerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructs the node and sets up bundling logic.
   * @param options The node options.
   */
  explicit AuvStatusBundlerNode(const rclcpp::NodeOptions& options);

 protected:
  /**
   * @brief Callback for incoming local odometry data.
   * @param msg The latest odometry message.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Callback for incoming pressure/depth data.
   * @param msg The latest depth odometry message.
   */
  void depthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Callback for incoming IMU orientation data.
   * @param msg The latest IMU message.
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<coug_interfaces::msg::AgentStatus>::SharedPtr status_pub_;

  // --- Parameters ---
  std::shared_ptr<auv_status_bundler_node::ParamListener> param_listener_;
  auv_status_bundler_node::Params params_;

  // --- State ---
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_depth_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
};

}  // namespace coug_comms
