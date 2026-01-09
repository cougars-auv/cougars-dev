// Copyright (c) 2026 BYU FRoSt Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_conversion_utils.cpp
 * @brief Unit tests for conversion_utils.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>

#include "coug_fgo/utils/conversion_utils.hpp"

/**
 * @brief Test conversion between ROS Point messages and GTSAM Point3 objects.
 *
 * Verifies bi-directional conversion accuracy:
 * - ROS `Point` (x, y, z) -> GTSAM `Point3`
 * - GTSAM `Point3` -> ROS `Point`
 */
TEST(ConversionUtilsTest, PointConversion) {
  // Case 1: ROS Point to GTSAM Point3
  geometry_msgs::msg::Point ros_point;
  ros_point.x = 1.0;
  ros_point.y = 2.0;
  ros_point.z = 3.0;

  gtsam::Point3 gtsam_point = coug_fgo::utils::toGtsam(ros_point);
  EXPECT_DOUBLE_EQ(gtsam_point.x(), 1.0);
  EXPECT_DOUBLE_EQ(gtsam_point.y(), 2.0);
  EXPECT_DOUBLE_EQ(gtsam_point.z(), 3.0);

  // Case 2: GTSAM Point3 to ROS Point
  geometry_msgs::msg::Point back_to_ros = coug_fgo::utils::toPointMsg(gtsam_point);
  EXPECT_DOUBLE_EQ(back_to_ros.x, 1.0);
  EXPECT_DOUBLE_EQ(back_to_ros.y, 2.0);
  EXPECT_DOUBLE_EQ(back_to_ros.z, 3.0);
}

/**
 * @brief Test conversion between ROS Vector3 messages and GTSAM Vector3 objects.
 *
 * Verifies bi-directional conversion accuracy:
 * - ROS `Vector3` -> GTSAM `Vector3`
 * - GTSAM `Vector3` -> ROS `Vector3`
 */
TEST(ConversionUtilsTest, Vector3Conversion) {
  // Case 1: ROS Vector3 to GTSAM Vector3
  geometry_msgs::msg::Vector3 ros_vec;
  ros_vec.x = 0.5;
  ros_vec.y = -0.5;
  ros_vec.z = 10.0;

  gtsam::Vector3 gtsam_vec = coug_fgo::utils::toGtsam(ros_vec);
  EXPECT_DOUBLE_EQ(gtsam_vec.x(), 0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.y(), -0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.z(), 10.0);

  // Case 2: GTSAM Vector3 to ROS Vector3
  geometry_msgs::msg::Vector3 back_to_ros = coug_fgo::utils::toVectorMsg(gtsam_vec);
  EXPECT_DOUBLE_EQ(back_to_ros.x, 0.5);
  EXPECT_DOUBLE_EQ(back_to_ros.y, -0.5);
  EXPECT_DOUBLE_EQ(back_to_ros.z, 10.0);
}

/**
 * @brief Test conversion between ROS Quaternion messages and GTSAM Rot3 objects.
 *
 * Verifies:
 * - Identity quaternion conversion.
 * - Non-trivial rotation (90 degrees about Z) conversion.
 * - Accuracy of the resulting rotation matrix (Rot3).
 */
TEST(ConversionUtilsTest, QuaternionConversion) {
  // Case 1: Identity rotation
  geometry_msgs::msg::Quaternion ros_quat;
  ros_quat.w = 1.0;
  ros_quat.x = 0.0;
  ros_quat.y = 0.0;
  ros_quat.z = 0.0;

  gtsam::Rot3 gtsam_rot = coug_fgo::utils::toGtsam(ros_quat);
  EXPECT_TRUE(gtsam_rot.equals(gtsam::Rot3::Identity()));

  geometry_msgs::msg::Quaternion back_to_ros = coug_fgo::utils::toQuatMsg(gtsam_rot);
  EXPECT_DOUBLE_EQ(back_to_ros.w, 1.0);
  EXPECT_DOUBLE_EQ(back_to_ros.x, 0.0);

  // Case 2: 90 degrees around Z axis
  ros_quat.w = 0.70710678;
  ros_quat.z = 0.70710678;
  gtsam_rot = coug_fgo::utils::toGtsam(ros_quat);
  EXPECT_NEAR(gtsam_rot.yaw(), M_PI_2, 1e-5);
}

/**
 * @brief Test conversion between ROS Pose messages and GTSAM Pose3 objects.
 *
 * Verifies that both position and orientation components are correctly
 * transferred during conversion in both directions.
 */
TEST(ConversionUtilsTest, PoseConversion) {
  // Case 1: ROS Pose to GTSAM Pose3
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = 5.0;
  ros_pose.position.y = 6.0;
  ros_pose.position.z = 7.0;
  ros_pose.orientation.w = 1.0;

  gtsam::Pose3 gtsam_pose = coug_fgo::utils::toGtsam(ros_pose);
  EXPECT_DOUBLE_EQ(gtsam_pose.x(), 5.0);
  EXPECT_DOUBLE_EQ(gtsam_pose.y(), 6.0);
  EXPECT_DOUBLE_EQ(gtsam_pose.z(), 7.0);
  EXPECT_TRUE(gtsam_pose.rotation().equals(gtsam::Rot3::Identity()));

  // Case 2: GTSAM Pose3 to ROS Pose
  geometry_msgs::msg::Pose back_to_ros = coug_fgo::utils::toPoseMsg(gtsam_pose);
  EXPECT_DOUBLE_EQ(back_to_ros.position.x, 5.0);
  EXPECT_DOUBLE_EQ(back_to_ros.orientation.w, 1.0);
}
