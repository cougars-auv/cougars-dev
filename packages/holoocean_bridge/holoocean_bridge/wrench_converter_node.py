# Copyright (c) 2026 BYU FROST Lab
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
from geometry_msgs.msg import WrenchStamped
from holoocean_interfaces.msg import AgentCommand


class WrenchConverterNode(Node):
    """
    Converts HoloOcean agent command messages back to WrenchStamped messages.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("wrench_converter_node")

        self.declare_parameter("input_topic", "/command/agent")
        self.declare_parameter("output_topic", "cmd_wrench")
        self.declare_parameter("wrench_frame", "base_link")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.wrench_frame = (
            self.get_parameter("wrench_frame").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            AgentCommand, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(WrenchStamped, output_topic, 10)

        self.get_logger().info(
            f"Wrench converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: AgentCommand):
        """
        Process agent command messages.

        :param msg: AgentCommand message containing thruster values.
        """
        cmd = msg.command

        # From the BlueROV2.cpp file
        GEO_FACTOR = 0.70710678
        fwd = (cmd[4] + cmd[5] + cmd[6] + cmd[7]) * GEO_FACTOR
        lat = (cmd[4] - cmd[5] + cmd[6] - cmd[7]) * GEO_FACTOR
        vert = (cmd[0] + cmd[1] + cmd[2] + cmd[3])

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.wrench_frame

        wrench_msg.wrench.force.x = fwd
        wrench_msg.wrench.force.y = lat
        wrench_msg.wrench.force.z = vert
        self.publisher.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    wrench_converter_node = WrenchConverterNode()
    try:
        rclpy.spin(wrench_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrench_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
