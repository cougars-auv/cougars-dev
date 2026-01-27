# Copyright (c) 2026 BYU FRoSt Lab
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
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class GpsOdomConverterNode(Node):
    """
    Transforms GPS Odometry data to the base_link frame.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("gps_odom_converter_node")

        self.declare_parameter("input_topic", "odometry/gps")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("base_frame", "base_link")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f"GPS Odom converter started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry):
        """
        Transform GPS Odometry data to the base_link frame.

        :param msg: Odometry message in the GPS frame.
        """
        try:
            t_gps_base = self.tf_buffer.lookup_transform(
                msg.child_frame_id, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            self.get_logger().error(
                f"Could not find transform from {msg.child_frame_id} "
                f"to {self.base_frame}",
                throttle_duration_sec=1.0,
            )
            return

        p_base_in_map = do_transform_pose(msg.pose.pose, t_gps_base)

        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.child_frame_id = self.base_frame
        out_msg.pose.pose = p_base_in_map
        out_msg.pose.covariance = msg.pose.covariance
        out_msg.twist = msg.twist

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_odom_converter_node = GpsOdomConverterNode()
    try:
        rclpy.spin(gps_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
