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

import cv2
import message_filters
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from coug_perception.utils.class_colors import class_color


class BoxOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__("box_overlay_node")

        self.declare_parameter("labels_file", "")
        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("image_topic", "camera/rgb/image_rect_color")
        self.declare_parameter("boxes_topic", "camera/boxes")
        self.declare_parameter("output_topic", "camera/boxes/image")

        with open(self.get_parameter("labels_file").value) as f:
            self._labels = {str(label): name for label, name in yaml.safe_load(f).items()}
        sync_slop_sec = self.get_parameter("sync_slop_sec").value
        image_topic = self.get_parameter("image_topic").value
        boxes_topic = self.get_parameter("boxes_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._image_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data
        )
        self._boxes_sub = message_filters.Subscriber(
            self, Detection2DArray, boxes_topic, qos_profile=qos_profile_system_default
        )

        self._time_sync = message_filters.ApproximateTimeSynchronizer(
            [self._image_sub, self._boxes_sub],
            queue_size=10,
            slop=sync_slop_sec,
        )
        self._time_sync.registerCallback(self._sync_callback)

        self._output_pub = self.create_publisher(Image, output_topic, qos_profile_system_default)

        self._bridge = CvBridge()

        self.get_logger().info("Initialization complete.")

    def _sync_callback(self, image_msg: Image, boxes_msg: Detection2DArray, /) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            return

        for detection in boxes_msg.detections:
            if not detection.results:
                continue
            name = self._labels.get(detection.results[0].hypothesis.class_id)
            if name is None:
                continue
            color = tuple(round(255 * c) for c in reversed(class_color(name)))

            center = detection.bbox.center.position
            half_width = detection.bbox.size_x / 2.0
            half_height = detection.bbox.size_y / 2.0
            top_left = (int(center.x - half_width), int(center.y - half_height))
            bottom_right = (int(center.x + half_width), int(center.y + half_height))

            cv2.rectangle(cv_image, top_left, bottom_right, color, 2)
            cv2.putText(
                cv_image,
                name,
                (top_left[0], top_left[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        overlay_msg = self._bridge.cv2_to_imgmsg(cv_image, "bgr8")
        overlay_msg.header = image_msg.header
        self._output_pub.publish(overlay_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    box_overlay_node = BoxOverlayNode()
    try:
        rclpy.spin(box_overlay_node)
    except KeyboardInterrupt:
        pass
    finally:
        box_overlay_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
