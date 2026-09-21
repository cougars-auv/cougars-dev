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
from sensor_msgs.msg import NavSatFix


class NavsatCovarianceNode(Node):
    def __init__(self) -> None:
        super().__init__("navsat_covariance_node")

        self.declare_parameter("position_noise_sigmas", [0.015, 0.015, 0.025])
        self.declare_parameter("input_topic", "gps/fix_gz")
        self.declare_parameter("output_topic", "gps/fix")

        self._position_noise_sigmas = self.get_parameter("position_noise_sigmas").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._input_sub = self.create_subscription(
            NavSatFix, input_topic, self._navsat_callback, qos_profile_system_default
        )
        # Reliable QoS to match SBG-SYSTEMS/sbg_ros2_driver
        self._output_pub = self.create_publisher(
            NavSatFix, output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def _navsat_callback(self, msg: NavSatFix) -> None:
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            msg.position_covariance[0] = self._position_noise_sigmas[0] ** 2
            msg.position_covariance[4] = self._position_noise_sigmas[1] ** 2
            msg.position_covariance[8] = self._position_noise_sigmas[2] ** 2
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._output_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    navsat_covariance_node = NavsatCovarianceNode()
    try:
        rclpy.spin(navsat_covariance_node)
    except KeyboardInterrupt:
        pass
    finally:
        navsat_covariance_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
