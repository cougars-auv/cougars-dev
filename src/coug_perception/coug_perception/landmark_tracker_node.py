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
from builtin_interfaces.msg import Time
from norfair import Detection, Tracker
from norfair.filter import OptimizedKalmanFilterFactory
from norfair.tracker import TrackedObject
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
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


class LandmarkTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("landmark_tracker_node")

        self.declare_parameter("distance_threshold", 1.0)
        self.declare_parameter("relabel_distance_threshold", 0.4)
        self.declare_parameter("initialization_delay", 10)
        self.declare_parameter("input_topic", "detections_3d_labeled")
        self.declare_parameter("output_topic", "landmarks")
        self.declare_parameter("marker_topic", "landmarks/markers")
        self.declare_parameter("map_frame", "map")

        self._distance_threshold = self.get_parameter("distance_threshold").value
        self._relabel_distance_threshold = self.get_parameter("relabel_distance_threshold").value
        initialization_delay = self.get_parameter("initialization_delay").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        self._map_frame = self.get_parameter("map_frame").value

        self._tracker = Tracker(
            distance_function=self._distance,
            distance_threshold=self._distance_threshold,
            hit_counter_max=initialization_delay + 1,
            initialization_delay=initialization_delay,
            filter_factory=OptimizedKalmanFilterFactory(
                R=1.0, Q=0.0, pos_variance=1.0, pos_vel_covariance=0.0, vel_variance=0.0
            ),
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._input_sub = self.create_subscription(
            Detection3DArray, input_topic, self._detections_callback, qos_profile_system_default
        )
        self._output_pub = self.create_publisher(
            Detection3DArray, output_topic, qos_profile_system_default
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, marker_topic, qos_profile_system_default
        )

        self._published_ids: set[int] = set()

        self.get_logger().info("Initialization complete.")

    def _detections_callback(self, msg: Detection3DArray) -> None:
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if not self._tf_buffer.can_transform(self._map_frame, msg.header.frame_id, stamp):
            stamp = rclpy.time.Time()
        try:
            map_T_sensor_tf = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, stamp
            )
        except TransformException as e:
            self.get_logger().warning(
                f"Failed to look up transform from '{msg.header.frame_id}' to '{self._map_frame}': {e}",
                throttle_duration_sec=1.0,
            )
            return

        detections = []
        for detection in msg.detections:
            pose = do_transform_pose(detection.bbox.center, map_T_sensor_tf)
            points = np.array([[pose.position.x, pose.position.y, pose.position.z]])
            class_id = detection.results[0].hypothesis.class_id
            bbox = BoundingBox3D(center=pose, size=detection.bbox.size)
            detections.append(Detection(points=points, data=(class_id, bbox)))

        # Keep all initialized landmarks
        for obj in self._tracker.tracked_objects:
            if not obj.is_initializing:
                obj.hit_counter += 1
                obj.point_hit_counter += 1

        self._tracker.update(detections=detections)

        # Merge new landmarks into existing ones they match
        landmarks = self._tracker.get_active_objects()
        kept = [obj for obj in landmarks if obj.id in self._published_ids]
        for obj in landmarks:
            if obj.id in self._published_ids:
                continue
            matches = [
                other
                for other in kept
                if self._distance(obj.last_detection, other) <= self._distance_threshold
            ]
            if not matches:
                kept.append(obj)
                self._published_ids.add(obj.id)
                continue
            other = matches[0]
            gain = other.filter.pos_variance / (other.filter.pos_variance + obj.filter.pos_variance)
            other.filter.x[:3] += gain * (obj.filter.x[:3] - other.filter.x[:3])
            other.filter.pos_variance *= 1.0 - gain
        self._tracker.tracked_objects = [
            obj for obj in self._tracker.tracked_objects if obj.is_initializing or obj in kept
        ]

        for obj in kept:
            position = obj.last_detection.data[1].center.position
            position.x, position.y, position.z = (float(v) for v in obj.estimate[0])

        self._publish_landmarks(msg.header.stamp)

    def _distance(self, detection: Detection, tracked_object: TrackedObject) -> float:
        detection_class, detection_bbox = detection.data
        track_class, track_bbox = tracked_object.last_detection.data
        if detection_class != track_class:
            distance = float(np.linalg.norm(detection.points - tracked_object.estimate))
            return distance if distance <= self._relabel_distance_threshold else math.inf

        # Match on footprint for same-class merges
        gaps = []
        for point_bbox, box_bbox in ((detection_bbox, track_bbox), (track_bbox, detection_bbox)):
            q = box_bbox.center.orientation
            corners = cv2.boxPoints(
                (
                    (box_bbox.center.position.x, box_bbox.center.position.y),
                    (box_bbox.size.x, box_bbox.size.y),
                    math.degrees(2.0 * math.atan2(q.z, q.w)),
                )
            )
            center = (point_bbox.center.position.x, point_bbox.center.position.y)
            gaps.append(-cv2.pointPolygonTest(corners, center, True))
        return float(max(min(gaps), 0.0))

    def _publish_landmarks(self, stamp: Time) -> None:
        landmarks_msg = Detection3DArray()
        landmarks_msg.header.stamp = stamp
        landmarks_msg.header.frame_id = self._map_frame

        markers_msg = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        markers_msg.markers.append(clear_marker)

        for obj in self._tracker.get_active_objects():
            class_id, bbox = obj.last_detection.data

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id
            hypothesis.hypothesis.score = 1.0

            detection = Detection3D()
            detection.header = landmarks_msg.header
            detection.id = str(obj.id)
            detection.bbox = bbox
            detection.results.append(hypothesis)
            landmarks_msg.detections.append(detection)

            box_marker = Marker()
            box_marker.header = landmarks_msg.header
            box_marker.ns = "landmarks"
            box_marker.id = obj.id
            box_marker.type = Marker.CUBE
            box_marker.pose = bbox.center
            box_marker.scale = bbox.size
            box_marker.color.r, box_marker.color.g, box_marker.color.b = class_color(class_id)
            box_marker.color.a = 0.8
            markers_msg.markers.append(box_marker)

            label_marker = Marker()
            label_marker.header = landmarks_msg.header
            label_marker.ns = "landmark_labels"
            label_marker.id = obj.id
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.pose.position.x = bbox.center.position.x
            label_marker.pose.position.y = bbox.center.position.y
            label_marker.pose.position.z = bbox.center.position.z + bbox.size.z / 2.0 + 0.3
            label_marker.scale.z = 0.4
            label_marker.color.r = label_marker.color.g = label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = f"{obj.id} {class_id}"
            markers_msg.markers.append(label_marker)

        self._output_pub.publish(landmarks_msg)
        self._marker_pub.publish(markers_msg)


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
