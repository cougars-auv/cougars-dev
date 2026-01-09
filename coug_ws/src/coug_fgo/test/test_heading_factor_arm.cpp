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
 * @file test_heading_factor_arm.cpp
 * @brief Unit tests for heading_factor_arm.hpp.
 * @author Nelson Durrant (w Gemini 3 Pro)
 * @date Jan 2026
 */

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <boost/bind/bind.hpp>

#include "coug_fgo/factors/heading_factor_arm.hpp"

/**
 * @brief Test the error evaluation logic of the CustomHeadingFactorArm.
 *
 * Computes rotation residual: `error = measured_rot - (body_rot * calibration_rot)`.
 *
 * Cases tested:
 * 1.  **Identity**: Everything aligned. Zero error.
 * 2.  **Sensor Rotation**: Sensor rotated 90 deg relative to body.
 *     - If body is Identity, Sensor output is 90 deg. Error should be zero.
 * 3.  **Error Magnitude**: Introduction of a small angular error (10 deg) to verify
 *     that the residual magnitude is correct.
 */
TEST(HeadingFactorArmTest, ErrorEvaluation) {
  gtsam::Key poseKey = gtsam::symbol_shorthand::X(1);
  gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  // Case 1: Identity alignment
  coug_fgo::factors::CustomHeadingFactorArm factor1(poseKey, gtsam::Rot3::Identity(),
    gtsam::Rot3::Identity(), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor1.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 2: Sensor rotated 90 deg wrt Base
  coug_fgo::factors::CustomHeadingFactorArm factor2(poseKey, gtsam::Rot3::Yaw(M_PI_2),
    gtsam::Rot3::Yaw(M_PI_2), model);
  EXPECT_TRUE(
    gtsam::assert_equal(
      gtsam::Vector3::Zero(),
      factor2.evaluateError(gtsam::Pose3::Identity()), 1e-9));

  // Case 3: 10 deg error check
  double angle = 0.174533;    // 10 deg
  gtsam::Vector error =
    factor2.evaluateError(gtsam::Pose3(gtsam::Rot3::Yaw(angle), gtsam::Point3()));
  EXPECT_NEAR(error[2], angle, 1e-5);
}

/**
 * @brief Verify Jacobians of the CustomHeadingFactorArm using numerical differentiation.
 *
 * Validates analytical derivatives for rotational error, ensuring proper handling of SE(3) manifolds.
 */
TEST(HeadingFactorArmTest, Jacobians) {
  coug_fgo::factors::CustomHeadingFactorArm factor(gtsam::symbol_shorthand::X(1),
    gtsam::Rot3::Ypr(0.5, 0.1, -0.1),
    gtsam::Rot3::Ypr(0.1, 0, 0), gtsam::noiseModel::Isotropic::Sigma(3, 0.1));
  gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(0.4, 0.05, -0.05), gtsam::Point3(1, 1, 1));

  gtsam::Matrix expectedH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    boost::bind(
      &coug_fgo::factors::CustomHeadingFactorArm::evaluateError, &factor,
      boost::placeholders::_1, boost::none), pose, 1e-5);

  gtsam::Matrix actualH;
  factor.evaluateError(pose, actualH);
  EXPECT_TRUE(gtsam::assert_equal(expectedH, actualH, 1e-5));
}
