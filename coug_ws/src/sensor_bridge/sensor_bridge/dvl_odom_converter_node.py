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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
from dvl_msgs.msg import DVLDR


class DvlOdomConverterNode(Node):
    """
    Converts DVL dead-reckoned data to Odometry.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_odom_converter_node")

        self.declare_parameter("input_topic", "dvl/position")
        self.declare_parameter("output_topic", "odometry/dvl")
        self.declare_parameter("child_frame_id", "dvl_link")

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            DVLDR, input_topic, self.listener_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f"DVL odom converter started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def listener_callback(self, msg: DVLDR):
        """
        Process dead-reckoned position data (DVLDR message).

        :param msg: DVLDR message containing dead reckoning data.
        """
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.child_frame_id = self.child_frame_id

        # Read DVLDR time for timestamp
        sec = int(msg.time)
        nanosec = int((msg.time - sec) * 1e9)
        odom.header.stamp.sec = sec
        odom.header.stamp.nanosec = nanosec

        # Convert from NED to ENU
        odom.pose.pose.position.x = msg.position.y
        odom.pose.pose.position.y = msg.position.x
        odom.pose.pose.position.z = -msg.position.z

        r_old = R.from_euler("xyz", [msg.roll, msg.pitch, msg.yaw], degrees=True)
        t_matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        r_new = R.from_matrix(t_matrix @ r_old.as_matrix())
        q_new = r_new.as_quat()

        odom.pose.pose.orientation = Quaternion(
            x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3]
        )

        # TODO: Look at TURTLMap covariance method
        var = msg.pos_std**2
        odom.pose.covariance[0] = var
        odom.pose.covariance[7] = var
        odom.pose.covariance[14] = var

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    dvl_odom_converter_node = DvlOdomConverterNode()
    try:
        rclpy.spin(dvl_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
