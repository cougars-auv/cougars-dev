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

"""Fill in the GPS covariance that Gazebo cannot report.

``gz.msgs.NavSat`` has no covariance field, so ros_gz_bridge always publishes
``COVARIANCE_TYPE_UNKNOWN`` with a zero matrix, which coug_fg's navsat_odom
rejects outright. The noise is configured in the URDF's ``<navsat>`` sensor, so
restate those standard deviations here and mark the result as known.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class NavsatCovarianceNode(Node):
    def __init__(self) -> None:
        super().__init__("navsat_covariance_node")

        self.declare_parameter("input_topic", "gps/fix_raw")
        self.declare_parameter("output_topic", "gps/fix")
        # Defaults match the <navsat> noise in rover.gazebo.xacro
        self.declare_parameter("horizontal_stddev", 0.015)
        self.declare_parameter("vertical_stddev", 0.025)

        horizontal = self.get_parameter("horizontal_stddev").value
        vertical = self.get_parameter("vertical_stddev").value
        self._covariance = [
            horizontal**2,
            0.0,
            0.0,
            0.0,
            horizontal**2,
            0.0,
            0.0,
            0.0,
            vertical**2,
        ]

        self._publisher = self.create_publisher(
            NavSatFix, self.get_parameter("output_topic").value, 10
        )
        self._subscription = self.create_subscription(
            NavSatFix,
            self.get_parameter("input_topic").value,
            self._callback,
            10,
        )

    def _callback(self, msg: NavSatFix) -> None:
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            msg.position_covariance = self._covariance
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = NavsatCovarianceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
