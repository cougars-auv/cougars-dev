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

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3D, Detection3DArray


class ClusterDetectionsNode(Node):
    def __init__(self) -> None:
        super().__init__("cluster_detections_node")

        self.declare_parameter("min_size", 0.05)
        self.declare_parameter("input_topic", "clusters/points")
        self.declare_parameter("output_topic", "detections_3d")

        self._min_size = self.get_parameter("min_size").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._input_sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_callback, qos_profile_system_default
        )
        self._output_pub = self.create_publisher(
            Detection3DArray, output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def _cloud_callback(self, msg: PointCloud2) -> None:
        points = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z", "intensity"], skip_nans=True
        )

        detections_msg = Detection3DArray()
        detections_msg.header = msg.header
        for cluster_id in np.unique(points[:, 3]):
            cluster = points[points[:, 3] == cluster_id, :3]
            (center_x, center_y), (length, width), angle = cv2.minAreaRect(
                np.ascontiguousarray(cluster[:, :2], dtype=np.float32)
            )
            if width > length:
                length, width, angle = width, length, angle + 90.0
            z_min, z_max = float(cluster[:, 2].min()), float(cluster[:, 2].max())
            yaw = math.radians(angle)

            detection = Detection3D()
            detection.header = msg.header
            detection.id = str(int(cluster_id))
            detection.bbox.center.position.x = center_x
            detection.bbox.center.position.y = center_y
            detection.bbox.center.position.z = (z_min + z_max) / 2.0
            detection.bbox.center.orientation.z = math.sin(yaw / 2.0)
            detection.bbox.center.orientation.w = math.cos(yaw / 2.0)
            detection.bbox.size.x = max(length, self._min_size)
            detection.bbox.size.y = max(width, self._min_size)
            detection.bbox.size.z = max(z_max - z_min, self._min_size)
            detections_msg.detections.append(detection)

        self._output_pub.publish(detections_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    cluster_detections_node = ClusterDetectionsNode()
    try:
        rclpy.spin(cluster_detections_node)
    except KeyboardInterrupt:
        pass
    finally:
        cluster_detections_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
