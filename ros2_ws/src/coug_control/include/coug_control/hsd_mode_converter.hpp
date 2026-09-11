// Copyright 2026 BYU FROST Lab
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

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "coug_control/hsd_mode_converter_parameters.hpp"
#include "coug_interfaces/msg/control_setpoint.hpp"
#include "coug_interfaces/msg/dvl_beam_list.hpp"

namespace coug_control {

class HsdModeConverterNode : public rclcpp::Node {
 public:
  explicit HsdModeConverterNode(const rclcpp::NodeOptions& options);

 private:
  // --- Callbacks ---
  void hsdCallback(const coug_interfaces::msg::ControlSetpoint::ConstSharedPtr& msg);

  void beamsCallback(const coug_interfaces::msg::DvlBeamList::ConstSharedPtr& msg);

  // --- Helpers ---
  auto convertToDepth(const coug_interfaces::msg::ControlSetpoint::ConstSharedPtr& msg) const
      -> coug_interfaces::msg::ControlSetpoint;

  // --- ROS Interfaces ---
  rclcpp::Subscription<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_sub_;
  rclcpp::Subscription<coug_interfaces::msg::DvlBeamList>::SharedPtr beams_sub_;
  rclcpp::Publisher<coug_interfaces::msg::ControlSetpoint>::SharedPtr hsd_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Parameters ---
  std::shared_ptr<hsd_mode_converter_node::ParamListener> param_listener_;
  hsd_mode_converter_node::Params params_;

  // --- State ---
  double seafloor_z_{0.0};
  bool has_seafloor_{false};
};

}  // namespace coug_control
