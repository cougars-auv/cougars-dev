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
 * @file auv_receiver.cpp
 * @brief Implementation of the AuvReceiverNode.
 * @author Nelson Durrant
 * @date June 2026
 */

#include "coug_comms/auv_receiver.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_comms {

using utils::MsgId;

AuvReceiverNode::AuvReceiverNode(const rclcpp::NodeOptions& options)
    : Node("auv_receiver_node", options), diagnostic_updater_(this) {
  RCLCPP_INFO(get_logger(), "Starting AUV Receiver Node...");

  param_listener_ =
      std::make_shared<auv_receiver_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  modem_rec_sub_ = create_subscription<seatrac_interfaces::msg::ModemRec>(
      params_.modem_rec_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&AuvReceiverNode::modemRecCallback, this, std::placeholders::_1));

  start_client_ = create_client<std_srvs::srv::Trigger>(params_.start_service);
  stop_client_ = create_client<std_srvs::srv::Trigger>(params_.stop_service);
  surface_client_ = create_client<std_srvs::srv::Trigger>(params_.surface_service);
  home_client_ = create_client<std_srvs::srv::Trigger>(params_.home_service);
  emergency_stop_client_ = create_client<std_srvs::srv::Trigger>(params_.emergency_stop_service);
  emergency_surface_client_ =
      create_client<std_srvs::srv::Trigger>(params_.emergency_surface_service);

  // --- ROS Diagnostics ---
  if (params_.publish_diagnostics) {
    std::string ns = this->get_namespace();
    std::string clean_ns = (ns == "/") ? "" : ns;
    diagnostic_updater_.setHardwareID(clean_ns + "/auv_receiver_node");

    std::string prefix = clean_ns.empty() ? "" : "[" + clean_ns + "] ";
    std::string cmd_task = prefix + "Service Status";
    diagnostic_updater_.add(cmd_task, this, &AuvReceiverNode::checkServiceStatus);
  }

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for acoustic services...");
}

void AuvReceiverNode::modemRecCallback(const seatrac_interfaces::msg::ModemRec::SharedPtr msg) {
  if (!msg->local_flag || msg->packet_len < 1) return;

  const auto id = static_cast<MsgId>(msg->packet_data[0]);
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
  switch (id) {
    case MsgId::SRV_START:
      client = start_client_;
      break;
    case MsgId::SRV_STOP:
      client = stop_client_;
      break;
    case MsgId::SRV_SURFACE:
      client = surface_client_;
      break;
    case MsgId::SRV_HOME:
      client = home_client_;
      break;
    case MsgId::SRV_EMERGENCY_STOP:
      client = emergency_stop_client_;
      break;
    case MsgId::SRV_EMERGENCY_SURFACE:
      client = emergency_surface_client_;
      break;
    default:
      return;
  }

  RCLCPP_INFO(get_logger(), "Received %s from beacon %d", utils::messageType(id).c_str(),
              msg->src_id);
  callService(client, id);
}

void AuvReceiverNode::callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client,
                                  MsgId cmd) {
  const std::string service = utils::messageType(cmd);
  if (!client->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "Service not available: %s", service.c_str());
    recordServiceResult(service, "ACOUSTIC", false);
    return;
  }
  client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, service](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        bool success = false;
        try {
          success = future.get()->success;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s; %s", service.c_str(), e.what());
        }
        recordServiceResult(service, "ACOUSTIC", success);
        if (success) {
          RCLCPP_INFO(get_logger(), "Service call succeeded: %s", service.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Service call failed: %s", service.c_str());
        }
      });
}

void AuvReceiverNode::recordServiceResult(const std::string& service, const std::string& transport,
                                          bool succeeded) {
  service_history_.push_back({service, transport, succeeded});
  if (service_history_.size() > static_cast<size_t>(params_.service_history_size)) {
    service_history_.pop_front();
  }
}

void AuvReceiverNode::checkServiceStatus(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (service_history_.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Waiting for first service.");
    return;
  }

  size_t index = 1;
  for (auto it = service_history_.rbegin(); it != service_history_.rend(); ++it) {
    stat.add(std::to_string(index++), it->service + " (" + it->transport + ")" +
                                          (it->succeeded ? ": succeeded" : ": failed"));
  }

  const ServiceResult& latest = service_history_.back();
  if (latest.succeeded) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, latest.service + " succeeded.");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, latest.service + " failed.");
  }
}

}  // namespace coug_comms

RCLCPP_COMPONENTS_REGISTER_NODE(coug_comms::AuvReceiverNode)
