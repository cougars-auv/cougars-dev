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
import message_filters
import numpy as np
import rclpy
import yaml
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import (  # type: ignore[attr-defined, unused-ignore]
    Buffer,
    TransformException,
    TransformListener,
)
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


class DetectionFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("detection_fusion_node")

        self.declare_parameter("labels_file", "")
        self.declare_parameter("sizes_file", "")
        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("min_iou", 0.1)
        self.declare_parameter("min_heading_ratio", 1.5)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("input_topic", "clusters/points")
        self.declare_parameter("boxes_topic", "camera/boxes")
        self.declare_parameter("camera_info_topic", "camera/rgb/camera_info")
        self.declare_parameter("output_topic", "detections_3d_labeled")
        self.declare_parameter("map_frame", "map")

        with open(self.get_parameter("labels_file").value) as f:
            self._labels = {str(label): name for label, name in yaml.safe_load(f).items()}
        with open(self.get_parameter("sizes_file").value) as f:
            self._sizes = {
                name: np.array([max(x, y), min(x, y), z])
                for name, (x, y, z) in yaml.safe_load(f).items()
            }
        sync_slop_sec = self.get_parameter("sync_slop_sec").value
        self._min_iou = self.get_parameter("min_iou").value
        self._min_heading_ratio = self.get_parameter("min_heading_ratio").value
        self._transform_timeout_sec = self.get_parameter("transform_timeout_sec").value
        input_topic = self.get_parameter("input_topic").value
        boxes_topic = self.get_parameter("boxes_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._map_frame = self.get_parameter("map_frame").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._input_sub = message_filters.Subscriber(
            self, PointCloud2, input_topic, qos_profile=qos_profile_system_default
        )
        self._boxes_sub = message_filters.Subscriber(
            self, Detection2DArray, boxes_topic, qos_profile=qos_profile_system_default
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data
        )

        self._time_sync = message_filters.ApproximateTimeSynchronizer(
            [self._input_sub, self._boxes_sub],
            queue_size=20,
            slop=sync_slop_sec,
        )
        self._time_sync.registerCallback(self._sync_callback)

        self._output_pub = self.create_publisher(
            Detection3DArray, output_topic, qos_profile_system_default
        )

        self._camera_model: PinholeCameraModel | None = None

        self.get_logger().info("Initialization complete.")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_model is None:
            camera_model = PinholeCameraModel()
            camera_model.from_camera_info(msg)
            self._camera_model = camera_model

    def _sync_callback(self, clusters_msg: PointCloud2, boxes_msg: Detection2DArray, /) -> None:
        if self._camera_model is None:
            return

        names, rects = [], []
        for box in boxes_msg.detections:
            if box.results and box.results[0].hypothesis.class_id in self._labels:
                names.append(self._labels[box.results[0].hypothesis.class_id])
                center = box.bbox.center.position
                half_x, half_y = box.bbox.size_x / 2.0, box.bbox.size_y / 2.0
                rects.append(
                    (center.x - half_x, center.y - half_y, center.x + half_x, center.y + half_y)
                )
        if not names:
            return
        box_rects = np.array(rects)
        box_areas = np.prod(box_rects[:, 2:] - box_rects[:, :2], axis=1)

        # Project the cluster points into the camera image
        camera_frame = self._camera_model.get_tf_frame()
        try:
            camera_T_sensor_tf = self._tf_buffer.lookup_transform_full(
                camera_frame,
                rclpy.time.Time.from_msg(boxes_msg.header.stamp),
                clusters_msg.header.frame_id,
                rclpy.time.Time.from_msg(clusters_msg.header.stamp),
                self._map_frame,
                timeout=rclpy.duration.Duration(seconds=self._transform_timeout_sec),
            )
        except TransformException as e:
            self.get_logger().warning(
                f"Failed to look up transform from '{clusters_msg.header.frame_id}' to '{camera_frame}': {e}",
                throttle_duration_sec=1.0,
            )
            return

        q = camera_T_sensor_tf.transform.rotation
        camera_R_sensor = Rotation.from_quat([q.x, q.y, q.z, q.w])
        camera_p_sensor = [
            camera_T_sensor_tf.transform.translation.x,
            camera_T_sensor_tf.transform.translation.y,
            camera_T_sensor_tf.transform.translation.z,
        ]
        projection = self._camera_model.projection_matrix()

        cloud = point_cloud2.read_points_numpy(
            clusters_msg, field_names=["x", "y", "z", "intensity"], skip_nans=True
        )
        sensor_p_cloud = cloud[:, :3].astype(float)
        camera_p_cloud = camera_R_sensor.apply(sensor_p_cloud) + camera_p_sensor
        pixels = camera_p_cloud @ projection[:, :3].T + projection[:, 3]
        pixels = pixels[:, :2] / pixels[:, 2:]
        pixels[camera_p_cloud[:, 2] <= 0.0] = np.nan
        ranges = np.linalg.norm(sensor_p_cloud[:, :2], axis=1)
        claimed = np.zeros(len(cloud), dtype=bool)

        labeled_msg = Detection3DArray()
        labeled_msg.header = clusters_msg.header
        for box in np.argsort(box_areas):
            # Match points to boxes they are in, smallest boxes first
            size = self._sizes[names[box]]
            inside = (
                ~claimed
                & (pixels >= box_rects[box, :2]).all(axis=1)
                & (pixels <= box_rects[box, 2:]).all(axis=1)
            )
            if not inside.any():
                continue

            # Keep the cluster that fills most of the box, trimmed to the object's size
            cluster_ids, counts = np.unique(cloud[inside, 3], return_counts=True)
            inside &= cloud[:, 3] == cluster_ids[counts.argmax()]
            part = inside & (ranges <= ranges[inside].min() + size[:2].max())
            claimed |= part
            sensor_p_part = sensor_p_cloud[part]

            # Gate detections on intersection over union
            iou = np.prod(pixels[part].max(axis=0) - pixels[part].min(axis=0)) / box_areas[box]
            if iou < self._min_iou:
                continue

            (center_x, center_y), (length, width), angle = cv2.minAreaRect(
                np.ascontiguousarray(sensor_p_part[:, :2], dtype=np.float32)
            )
            if width > length:
                angle += 90.0
            yaw = math.radians(angle)
            center = np.array([center_x, center_y])

            # Trust heading only from views long enough to show it
            heading_known = (
                size[1] >= size[0] or max(length, width) >= self._min_heading_ratio * size[1]
            )
            if heading_known:
                normal = np.array([-math.sin(yaw), math.cos(yaw)])
                hidden = max(size[1] - min(length, width), 0.0)
                center += np.sign(normal @ center) * hidden / 2.0 * normal

            detection = Detection3D()
            detection.header = clusters_msg.header
            detection.bbox.center.position.x = float(center[0])
            detection.bbox.center.position.y = float(center[1])
            detection.bbox.center.position.z = float(sensor_p_part[:, 2].max() - size[2] / 2.0)
            detection.bbox.center.orientation.z = math.sin(yaw / 2.0)
            detection.bbox.center.orientation.w = math.cos(yaw / 2.0)
            detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = (
                float(v) for v in size
            )

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = names[box]
            hypothesis.hypothesis.score = float(iou)
            hypothesis.pose.pose = detection.bbox.center
            if not heading_known:
                hypothesis.pose.covariance[35] = math.inf
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
