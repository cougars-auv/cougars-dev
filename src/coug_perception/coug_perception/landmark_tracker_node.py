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
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import (  # type: ignore[attr-defined, unused-ignore]
    Buffer,
    TransformException,
    TransformListener,
)
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from coug_perception.utils.class_colors import class_color


@dataclass
class Landmark:
    id: int
    class_id: str
    bbox: BoundingBox3D
    class_scores: dict[str, float] = field(default_factory=dict)
    hits: int = 1
    misses: int = 0


class LandmarkTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("landmark_tracker_node")

        self.declare_parameter("distance_threshold", 0.4)
        self.declare_parameter("pose_gain", 0.4)
        self.declare_parameter("class_gain", 0.2)
        self.declare_parameter("min_hits", 10)
        self.declare_parameter("max_misses", 10)
        self.declare_parameter("max_unseen", 30)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("transform_timeout_sec", 0.1)
        self.declare_parameter("input_topic", "detections_3d_labeled")
        self.declare_parameter("camera_info_topic", "camera/rgb/camera_info")
        self.declare_parameter("output_topic", "landmarks")
        self.declare_parameter("marker_topic", "landmarks/markers")
        self.declare_parameter("label_topic", "landmarks/labels")
        self.declare_parameter("map_frame", "map")

        self._distance_threshold = self.get_parameter("distance_threshold").value
        self._pose_gain = self.get_parameter("pose_gain").value
        self._class_gain = self.get_parameter("class_gain").value
        self._min_hits = self.get_parameter("min_hits").value
        self._max_misses = self.get_parameter("max_misses").value
        self._max_unseen = self.get_parameter("max_unseen").value
        self._max_range = self.get_parameter("max_range").value
        self._transform_timeout_sec = self.get_parameter("transform_timeout_sec").value
        input_topic = self.get_parameter("input_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        output_topic = self.get_parameter("output_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        label_topic = self.get_parameter("label_topic").value
        self._map_frame = self.get_parameter("map_frame").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._input_sub = self.create_subscription(
            Detection3DArray, input_topic, self._detections_callback, qos_profile_system_default
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data
        )
        self._output_pub = self.create_publisher(
            Detection3DArray, output_topic, qos_profile_system_default
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, marker_topic, qos_profile_system_default
        )
        self._label_pub = self.create_publisher(
            MarkerArray, label_topic, qos_profile_system_default
        )

        self._landmarks: list[Landmark] = []
        self._next_id = 1
        self._camera_model: PinholeCameraModel | None = None

        self.get_logger().info("Initialization complete.")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._camera_model is None:
            camera_model = PinholeCameraModel()
            camera_model.from_camera_info(msg)
            self._camera_model = camera_model

    def _detections_callback(self, msg: Detection3DArray) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        timeout = rclpy.duration.Duration(seconds=self._transform_timeout_sec)
        try:
            map_T_sensor_tf = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, stamp, timeout=timeout
            )
        except TransformException as e:
            self.get_logger().warning(
                f"Failed to look up transform from '{msg.header.frame_id}' to '{self._map_frame}': {e}",
                throttle_duration_sec=1.0,
            )
            return

        # Transform detections into the map frame
        detections, heading_known = [], []
        for detection in msg.detections:
            pose = do_transform_pose(detection.bbox.center, map_T_sensor_tf)
            bbox = BoundingBox3D(center=pose, size=detection.bbox.size)
            result = detection.results[0]
            detections.append(Landmark(0, result.hypothesis.class_id, bbox))
            heading_known.append(not math.isinf(result.pose.covariance[35]))

        # Match each detection to at most one existing landmark
        costs = np.array(
            [
                [self._distance(detection, landmark) for landmark in self._landmarks]
                for detection in detections
            ]
        ).reshape(len(detections), len(self._landmarks))
        rows, cols = linear_sum_assignment(np.minimum(costs, 1e6))
        matches = {
            r: c for r, c in zip(rows, cols, strict=True) if costs[r, c] <= self._distance_threshold
        }

        # Find the landmarks in the camera's view and range
        in_view: set[int] = set()
        if self._camera_model is not None:
            camera_frame = self._camera_model.get_tf_frame()
            try:
                camera_T_map_tf = self._tf_buffer.lookup_transform(
                    camera_frame, self._map_frame, stamp, timeout=timeout
                )
            except TransformException as e:
                self.get_logger().warning(
                    f"Failed to look up transform from '{self._map_frame}' to '{camera_frame}': {e}",
                    throttle_duration_sec=1.0,
                )
            else:
                q = camera_T_map_tf.transform.rotation
                camera_R_map = Rotation.from_quat([q.x, q.y, q.z, q.w])
                camera_p_map = [
                    camera_T_map_tf.transform.translation.x,
                    camera_T_map_tf.transform.translation.y,
                    camera_T_map_tf.transform.translation.z,
                ]
                width, height = self._camera_model.full_resolution()
                for landmark in self._landmarks:
                    p = landmark.bbox.center.position
                    camera_p_landmark = camera_R_map.apply([p.x, p.y, p.z]) + camera_p_map
                    if not 0.0 < camera_p_landmark[2] <= self._max_range:
                        continue
                    u, v = self._camera_model.project_3d_to_pixel(camera_p_landmark)
                    if 0.0 <= u < width and 0.0 <= v < height:
                        in_view.add(landmark.id)

        # Count misses for missing landmarks and update existing matched ones
        for landmark in self._landmarks:
            if landmark.hits < self._min_hits or landmark.id in in_view:
                landmark.misses += 1
        for r, c in matches.items():
            landmark, detection = self._landmarks[c], detections[r]
            if heading_known[r]:
                self._smooth_pose(landmark, detection.bbox)
            for class_id in landmark.class_scores:
                landmark.class_scores[class_id] *= 1.0 - self._class_gain
            landmark.class_scores[detection.class_id] = (
                landmark.class_scores.get(detection.class_id, 0.0) + self._class_gain
            )
            landmark.class_id = max(landmark.class_scores, key=landmark.class_scores.__getitem__)
            landmark.hits += 1
            landmark.misses = 0

        # Start new landmark from unmatched detections
        for r, detection in enumerate(detections):
            if r not in matches and heading_known[r]:
                detection.id = self._next_id
                detection.class_scores[detection.class_id] = 1.0
                self._next_id += 1
                self._landmarks.append(detection)

        # Drop unseen landmarks and check for duplicates
        kept: list[Landmark] = []
        for landmark in self._landmarks:
            if landmark.hits < self._min_hits:
                if landmark.misses <= self._max_misses:
                    kept.append(landmark)
                continue
            for other in kept:
                if other.hits >= self._min_hits and (
                    min(self._distance(landmark, other), self._distance(other, landmark))
                    <= self._distance_threshold
                ):
                    if landmark.misses < other.misses:
                        self._smooth_pose(other, landmark.bbox)
                        other.misses = landmark.misses
                    break
            else:
                if landmark.misses <= self._max_unseen:
                    kept.append(landmark)
        self._landmarks = kept

        landmarks_msg = Detection3DArray()
        landmarks_msg.header.stamp = msg.header.stamp
        landmarks_msg.header.frame_id = self._map_frame

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers_msg = MarkerArray(markers=[clear_marker])
        labels_msg = MarkerArray(markers=[clear_marker])

        for landmark in self._landmarks:
            if landmark.hits < self._min_hits:
                continue
            bbox = landmark.bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = landmark.class_id
            hypothesis.hypothesis.score = 1.0

            detection = Detection3D()
            detection.header = landmarks_msg.header
            detection.id = str(landmark.id)
            detection.bbox = bbox
            detection.results.append(hypothesis)
            landmarks_msg.detections.append(detection)

            box_marker = Marker()
            box_marker.header = landmarks_msg.header
            box_marker.ns = "landmarks"
            box_marker.id = landmark.id
            box_marker.type = Marker.LINE_LIST
            box_marker.pose = bbox.center
            box_marker.scale.x = 0.03
            box_marker.color.r, box_marker.color.g, box_marker.color.b = class_color(
                landmark.class_id
            )
            box_marker.color.a = 1.0
            corners = [
                Point(x=sx * bbox.size.x / 2.0, y=sy * bbox.size.y / 2.0, z=sz * bbox.size.z / 2.0)
                for sx in (-1.0, 1.0)
                for sy in (-1.0, 1.0)
                for sz in (-1.0, 1.0)
            ]
            for i in range(8):
                for j in range(i + 1, 8):
                    if (i ^ j).bit_count() == 1:
                        box_marker.points += [corners[i], corners[j]]
            markers_msg.markers.append(box_marker)

            label_marker = Marker()
            label_marker.header = landmarks_msg.header
            label_marker.ns = "landmark_labels"
            label_marker.id = landmark.id
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.pose.position.x = bbox.center.position.x
            label_marker.pose.position.y = bbox.center.position.y
            label_marker.pose.position.z = bbox.center.position.z + bbox.size.z / 2.0 + 0.2
            label_marker.scale.z = 0.2
            label_marker.color.r = label_marker.color.g = label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = f"{landmark.id} {landmark.class_id}"
            class_share = landmark.class_scores[landmark.class_id] / sum(
                landmark.class_scores.values()
            )
            if class_share < 0.995:
                label_marker.text += f" {class_share:.0%}"
            if landmark.misses > 0:
                label_marker.text += f" missed {landmark.misses}"
            labels_msg.markers.append(label_marker)

        self._output_pub.publish(landmarks_msg)
        self._marker_pub.publish(markers_msg)
        self._label_pub.publish(labels_msg)

    def _smooth_pose(self, landmark: Landmark, bbox: BoundingBox3D) -> None:
        previous, current = landmark.bbox.center, bbox.center
        current.position.x += (1.0 - self._pose_gain) * (previous.position.x - current.position.x)
        current.position.y += (1.0 - self._pose_gain) * (previous.position.y - current.position.y)
        current.position.z += (1.0 - self._pose_gain) * (previous.position.z - current.position.z)

        previous_yaw = 2.0 * math.atan2(previous.orientation.z, previous.orientation.w)
        yaw = 2.0 * math.atan2(current.orientation.z, current.orientation.w)
        step = (yaw - previous_yaw + math.pi / 2.0) % math.pi - math.pi / 2.0
        yaw = previous_yaw + self._pose_gain * step
        current.orientation.z = math.sin(yaw / 2.0)
        current.orientation.w = math.cos(yaw / 2.0)
        landmark.bbox = bbox

    def _distance(self, detection: Landmark, landmark: Landmark) -> float:
        # Center-to-center comparison within a class family
        if detection.class_id != landmark.class_id:
            if detection.class_id.rsplit("_", 1)[0] != landmark.class_id.rsplit("_", 1)[0]:
                return math.inf
            p, q = detection.bbox.center.position, landmark.bbox.center.position
            return math.dist((p.x, p.y, p.z), (q.x, q.y, q.z))

        # Footprint-to-center comparison within the same class
        bbox, q = landmark.bbox, landmark.bbox.center.orientation
        footprint = cv2.boxPoints(
            (
                (bbox.center.position.x, bbox.center.position.y),
                (bbox.size.x, bbox.size.y),
                math.degrees(2.0 * math.atan2(q.z, q.w)),
            )
        )
        point = (detection.bbox.center.position.x, detection.bbox.center.position.y)
        return float(max(-cv2.pointPolygonTest(footprint, point, True), 0.0))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    landmark_tracker_node = LandmarkTrackerNode()
    try:
        rclpy.spin(landmark_tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        landmark_tracker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
