# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import Imu

_UNKNOWN_COVARIANCE = -1.0


class ImuCovarianceNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_covariance_node")

        self.declare_parameter("orientation_noise_sigmas", [0.05, 0.05, 0.05])
        self.declare_parameter("enable_orientation", False)
        self.declare_parameter("input_topic", "camera/imu/data_gz")
        self.declare_parameter("output_topic", "camera/imu/data_raw")

        self._orientation_noise_sigmas = self.get_parameter("orientation_noise_sigmas").value
        self._enable_orientation = self.get_parameter("enable_orientation").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._input_sub = self.create_subscription(
            Imu, input_topic, self._imu_callback, qos_profile_system_default
        )
        # Reliable QoS to match stereolabs/zed-ros2-wrapper
        self._output_pub = self.create_publisher(Imu, output_topic, qos_profile_system_default)

        self.get_logger().info("Initialization complete.")

    def _imu_callback(self, msg: Imu) -> None:
        if self._enable_orientation:
            msg.orientation_covariance[0] = self._orientation_noise_sigmas[0] ** 2
            msg.orientation_covariance[4] = self._orientation_noise_sigmas[1] ** 2
            msg.orientation_covariance[8] = self._orientation_noise_sigmas[2] ** 2
        else:
            msg.orientation_covariance[0] = _UNKNOWN_COVARIANCE

        self._output_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    imu_covariance_node = ImuCovarianceNode()
    try:
        rclpy.spin(imu_covariance_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_covariance_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
