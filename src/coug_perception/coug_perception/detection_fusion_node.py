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
import yaml
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (  # type: ignore[attr-defined, unused-ignore]
    Buffer,
    TransformException,
    TransformListener,
)
from vision_msgs.msg import (
    Detection2DArray,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


class DetectionFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("detection_fusion_node")

        self.declare_parameter("labels_file", "")
        self.declare_parameter("max_boxes_age_sec", 0.3)
        self.declare_parameter("input_topic", "detections_3d")
        self.declare_parameter("boxes_topic", "camera/boxes")
        self.declare_parameter("camera_info_topic", "camera/rgb/camera_info")
        self.declare_parameter("output_topic", "detections_3d_labeled")

        with open(self.get_parameter("labels_file").value) as f:
            self._labels = {str(label): name for label, name in yaml.safe_load(f).items()}
        self._max_boxes_age = Duration(seconds=self.get_parameter("max_boxes_age_sec").value)
        input_topic = self.get_parameter("input_topic").value
        boxes_topic = self.get_parameter("boxes_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._input_sub = self.create_subscription(
            Detection3DArray, input_topic, self._detections_callback, qos_profile_system_default
        )
        self._boxes_sub = self.create_subscription(
            Detection2DArray, boxes_topic, self._boxes_callback, qos_profile_system_default
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data
        )
        self._output_pub = self.create_publisher(
            Detection3DArray, output_topic, qos_profile_system_default
        )

        self._camera_model: PinholeCameraModel | None = None
        self._boxes_msg: Detection2DArray | None = None

        self.get_logger().info("Initialization complete.")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_model is None:
            camera_model = PinholeCameraModel()
            camera_model.from_camera_info(msg)
            self._camera_model = camera_model

    def _boxes_callback(self, msg: Detection2DArray) -> None:
        self._boxes_msg = msg

    def _detections_callback(self, msg: Detection3DArray) -> None:
        if (
            self._camera_model is None
            or self._boxes_msg is None
            or self.get_clock().now() - rclpy.time.Time.from_msg(self._boxes_msg.header.stamp)
            > self._max_boxes_age
        ):
            return

        boxes = [
            (self._labels[box.results[0].hypothesis.class_id], box.bbox)
            for box in self._boxes_msg.detections
            if box.results and box.results[0].hypothesis.class_id in self._labels
        ]

        camera_frame = self._camera_model.get_tf_frame()
        try:
            camera_T_sensor_tf = self._tf_buffer.lookup_transform(
                camera_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warning(
                f"Failed to look up transform from '{msg.header.frame_id}' to '{camera_frame}': {e}",
                throttle_duration_sec=1.0,
            )
            return

        labeled_msg = Detection3DArray()
        labeled_msg.header = msg.header
        for detection in msg.detections:
            center = PointStamped()
            center.point = detection.bbox.center.position
            p = do_transform_point(center, camera_T_sensor_tf).point
            if p.z <= 0.0:
                continue
            u, v = self._camera_model.project_3d_to_pixel((p.x, p.y, p.z))

            containing = [
                (bbox.size_x * bbox.size_y, name)
                for name, bbox in boxes
                if abs(u - bbox.center.position.x) <= bbox.size_x / 2.0
                and abs(v - bbox.center.position.y) <= bbox.size_y / 2.0
            ]
            if not containing:
                continue

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = min(containing)[1]
            hypothesis.hypothesis.score = 1.0
            detection.results = [hypothesis]
            labeled_msg.detections.append(detection)

        self._output_pub.publish(labeled_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    detection_fusion_node = DetectionFusionNode()
    try:
        rclpy.spin(detection_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        detection_fusion_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
