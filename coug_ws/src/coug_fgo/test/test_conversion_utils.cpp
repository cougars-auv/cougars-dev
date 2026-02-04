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
 * Cases tested:
 * 1.  **ROS -> GTSAM**: Converts ROS Point msg to GTSAM Point3.
 * 2.  **GTSAM -> ROS**: Converts GTSAM Point3 to ROS Point msg.
 */
TEST(ConversionUtilsTest, PointConversion) {
  geometry_msgs::msg::Point ros_point;
  ros_point.x = 1.0;
  ros_point.y = 2.0;
  ros_point.z = 3.0;
  gtsam::Point3 gtsam_point = coug_fgo::utils::toGtsam(ros_point);
  EXPECT_DOUBLE_EQ(gtsam_point.x(), 1.0);
  EXPECT_DOUBLE_EQ(gtsam_point.y(), 2.0);
  EXPECT_DOUBLE_EQ(gtsam_point.z(), 3.0);

  geometry_msgs::msg::Point back_to_ros = coug_fgo::utils::toPointMsg(gtsam_point);
  EXPECT_DOUBLE_EQ(back_to_ros.x, 1.0);
  EXPECT_DOUBLE_EQ(back_to_ros.y, 2.0);
  EXPECT_DOUBLE_EQ(back_to_ros.z, 3.0);
}

/**
 * @brief Test conversion between ROS Vector3 messages and GTSAM Vector3 objects.
 *
 * Cases tested:
 * 1.  **ROS -> GTSAM**: Converts ROS Vector3 msg to GTSAM Vector3.
 * 2.  **GTSAM -> ROS**: Converts GTSAM Vector3 to ROS Vector3 msg.
 */
TEST(ConversionUtilsTest, Vector3Conversion) {
  geometry_msgs::msg::Vector3 ros_vec;
  ros_vec.x = 0.5;
  ros_vec.y = -0.5;
  ros_vec.z = 10.0;
  gtsam::Vector3 gtsam_vec = coug_fgo::utils::toGtsam(ros_vec);
  EXPECT_DOUBLE_EQ(gtsam_vec.x(), 0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.y(), -0.5);
  EXPECT_DOUBLE_EQ(gtsam_vec.z(), 10.0);

  geometry_msgs::msg::Vector3 back_to_ros = coug_fgo::utils::toVectorMsg(gtsam_vec);
  EXPECT_DOUBLE_EQ(back_to_ros.x, 0.5);
  EXPECT_DOUBLE_EQ(back_to_ros.y, -0.5);
  EXPECT_DOUBLE_EQ(back_to_ros.z, 10.0);
}

/**
 * @brief Test conversion between ROS Quaternion messages and GTSAM Rot3 objects.
 *
 * Cases tested:
 * 1.  **Identity**: Conversion of identity quaternion.
 * 2.  **Rotation**: Conversion of 90-degree Z-rotation quaternion.
 */
TEST(ConversionUtilsTest, QuaternionConversion) {
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

  ros_quat.w = 0.70710678;
  ros_quat.z = 0.70710678;
  gtsam_rot = coug_fgo::utils::toGtsam(ros_quat);
  EXPECT_NEAR(gtsam_rot.yaw(), M_PI_2, 1e-5);
}

/**
 * @brief Test conversion between ROS Pose messages and GTSAM Pose3 objects.
 *
 * Cases tested:
 * 1.  **ROS -> GTSAM**: Converts ROS Pose msg to GTSAM Pose3.
 * 2.  **GTSAM -> ROS**: Converts GTSAM Pose3 to ROS Pose msg.
 */
TEST(ConversionUtilsTest, PoseConversion) {
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

  geometry_msgs::msg::Pose back_to_ros = coug_fgo::utils::toPoseMsg(gtsam_pose);
  EXPECT_DOUBLE_EQ(back_to_ros.position.x, 5.0);
  EXPECT_DOUBLE_EQ(back_to_ros.orientation.w, 1.0);
}

/**
 * @brief Test conversion between ROS Transform messages and GTSAM Pose3 objects.
 *
 * Cases tested:
 * 1.  **ROS -> GTSAM**: Converts ROS Transform msg to GTSAM Pose3.
 */
TEST(ConversionUtilsTest, TransformConversion) {
  geometry_msgs::msg::Transform ros_tf;
  ros_tf.translation.x = 10.0;
  ros_tf.translation.y = 20.0;
  ros_tf.translation.z = 30.0;
  ros_tf.rotation.w = 0.70710678;
  ros_tf.rotation.z = 0.70710678;

  gtsam::Pose3 gtsam_pose = coug_fgo::utils::toGtsam(ros_tf);
  EXPECT_DOUBLE_EQ(gtsam_pose.x(), 10.0);
  EXPECT_DOUBLE_EQ(gtsam_pose.y(), 20.0);
  EXPECT_DOUBLE_EQ(gtsam_pose.z(), 30.0);
  EXPECT_NEAR(gtsam_pose.rotation().yaw(), M_PI_2, 1e-4);
}

/**
 * @brief Test conversion between ROS Wrench messages and GTSAM Vector6 objects.
 *
 * Cases tested:
 * 1.  **ROS -> GTSAM**: Converts ROS Wrench msg to GTSAM Vector6 (Force/Torque).
 */
TEST(ConversionUtilsTest, WrenchConversion) {
  geometry_msgs::msg::Wrench ros_wrench;
  ros_wrench.force.x = 1.0;
  ros_wrench.force.y = 2.0;
  ros_wrench.force.z = 3.0;
  ros_wrench.torque.x = 0.1;
  ros_wrench.torque.y = 0.2;
  ros_wrench.torque.z = 0.3;

  gtsam::Vector6 gtsam_wrench = coug_fgo::utils::toGtsam(ros_wrench);
  EXPECT_DOUBLE_EQ(gtsam_wrench(0), 1.0);
  EXPECT_DOUBLE_EQ(gtsam_wrench(1), 2.0);
  EXPECT_DOUBLE_EQ(gtsam_wrench(2), 3.0);
  EXPECT_DOUBLE_EQ(gtsam_wrench(3), 0.1);
  EXPECT_DOUBLE_EQ(gtsam_wrench(4), 0.2);
  EXPECT_DOUBLE_EQ(gtsam_wrench(5), 0.3);
}

/**
 * @brief Test conversion of ROS Covariance arrays to GTSAM Matrices.
 *
 * Cases tested:
 * 1.  **3x3 -> Matrix33**: Converts std::array<double, 9> to GTSAM Matrix33.
 * 2.  **6x6 -> Matrix66**: Converts std::array<double, 36> to GTSAM Matrix66.
 */
TEST(ConversionUtilsTest, CovarianceConversion) {
  std::array<double, 9> cov3x3;
  for (int i = 0; i < 9; ++i) {
    cov3x3[i] = static_cast<double>(i);
  }

  gtsam::Matrix33 m3 = coug_fgo::utils::toGtsam(cov3x3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(m3(i, j), static_cast<double>(i * 3 + j));
    }
  }

  std::array<double, 36> cov6x6;
  for (int i = 0; i < 36; ++i) {
    cov6x6[i] = static_cast<double>(i);
  }

  gtsam::Matrix66 m6 = coug_fgo::utils::toGtsam(cov6x6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_DOUBLE_EQ(m6(i, j), static_cast<double>(i * 6 + j));
    }
  }
}
