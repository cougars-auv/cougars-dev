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
 * @file auv_status_bundler.cpp
 * @brief Implementation of the AuvStatusBundlerNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_status_bundler.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_comms {

AuvStatusBundlerNode::AuvStatusBundlerNode(const rclcpp::NodeOptions& options)
    : Node("auv_status_bundler_node", options) {
  RCLCPP_INFO(get_logger(), "Starting AUV Status Bundler Node...");

  param_listener_ =
      std::make_shared<auv_status_bundler_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::odomCallback, this, std::placeholders::_1));

  depth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.depth_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::depthCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvStatusBundlerNode::imuCallback, this, std::placeholders::_1));

  status_pub_ = create_publisher<coug_interfaces::msg::AgentStatus>(params_.status_topic,
                                                                    rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for sensor data to bundle...");
}

void AuvStatusBundlerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_odom_ = msg;

  coug_interfaces::msg::AgentStatus status;
  status.header.stamp = msg->header.stamp;

  status.local_odometry = msg->pose.pose;
  status.odometry_covariance = msg->pose.covariance;

  if (last_depth_) {
    status.pressure_depth = last_depth_->pose.pose.position.z;
  } else {
    status.pressure_depth = 0.0;
  }

  if (last_imu_) {
    status.imu_orientation = last_imu_->orientation;
  } else {
    status.imu_orientation.w = 1.0;
  }

  status_pub_->publish(status);
}

void AuvStatusBundlerNode::depthCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  last_depth_ = msg;
}

void AuvStatusBundlerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  last_imu_ = msg;
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvStatusBundlerNode)
