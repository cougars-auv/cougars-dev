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

#include "coug_control/hsd_mode_converter.hpp"

#include <algorithm>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "coug_control/hsd_mode_converter_parameters.hpp"
#include "coug_interfaces/msg/control_setpoint.hpp"
#include "coug_interfaces/msg/dvl_beam_list.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace coug_control {

using coug_interfaces::msg::ControlSetpoint;
using coug_interfaces::msg::DvlBeamList;

HsdModeConverterNode::HsdModeConverterNode(const rclcpp::NodeOptions& options)
    : Node("hsd_mode_converter_node", options) {
  param_listener_ =
      std::make_shared<hsd_mode_converter_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  hsd_sub_ = create_subscription<ControlSetpoint>(
      params_.hsd_topic, rclcpp::SystemDefaultsQoS(),
      [this](const ControlSetpoint::ConstSharedPtr& msg) { hsdCallback(msg); });

  beams_sub_ = create_subscription<DvlBeamList>(
      params_.beams_topic, rclcpp::SystemDefaultsQoS(),
      [this](const DvlBeamList::ConstSharedPtr& msg) { beamsCallback(msg); });

  hsd_pub_ =
      create_publisher<ControlSetpoint>(params_.output_hsd_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void HsdModeConverterNode::hsdCallback(const ControlSetpoint::ConstSharedPtr& msg) {
  if (msg->mode == ControlSetpoint::ALTITUDE) {
    hsd_pub_->publish(convertToDepth(msg));
  } else {
    hsd_pub_->publish(*msg);
  }
}

void HsdModeConverterNode::beamsCallback(const DvlBeamList::ConstSharedPtr& msg) {
  if (!msg->altitude_valid) {
    return;
  }

  const std::string& dvl_frame = msg->header.frame_id;

  geometry_msgs::msg::TransformStamped map_T_dvl_tf;
  try {
    map_T_dvl_tf = tf_buffer_->lookupTransform(params_.map_frame, dvl_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform %s to %s: %s",
                         dvl_frame.c_str(), params_.map_frame.c_str(), ex.what());
    return;
  }

  // Transform the seafloor point (along the DVL z-axis) into the map frame
  geometry_msgs::msg::Pose dvl_T_seafloor;
  dvl_T_seafloor.position.z = -msg->altitude;

  geometry_msgs::msg::Pose map_T_seafloor;
  tf2::doTransform(dvl_T_seafloor, map_T_seafloor, map_T_dvl_tf);

  seafloor_z_ = map_T_seafloor.position.z;
  has_seafloor_ = true;
}

auto HsdModeConverterNode::convertToDepth(const ControlSetpoint::ConstSharedPtr& msg) const
    -> ControlSetpoint {
  if (!has_seafloor_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Received altitude setpoint with no seafloor estimate. "
                         "Holding at the surface.");
  }

  ControlSetpoint hsd_msg = *msg;
  hsd_msg.mode = ControlSetpoint::DEPTH;
  hsd_msg.depth = has_seafloor_ ? std::min(seafloor_z_ + msg->depth, 0.0) : 0.0;
  return hsd_msg;
}

}  // namespace coug_control

RCLCPP_COMPONENTS_REGISTER_NODE(coug_control::HsdModeConverterNode)
