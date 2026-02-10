// Copyright (c) 2026 BYU FROST Lab
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
 * @file mag_factor.hpp
 * @brief GTSAM factor for magnetometer measurements with a lever arm.
 * @author Nelson Durrant
 * @date Jan 2026
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace coug_fgo::factors
{

/**
 * @class MagFactorArm
 * @brief GTSAM factor for 3D magnetometer measurements with a lever arm.
 *
 * This factor constrains the 3D orientation of the AUV based on magnetometer measurements,
 * accounting for the rotation between the AUV base and the sensor.
 */
class MagFactorArm : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
  gtsam::Point3 measured_field_sensor_;
  gtsam::Point3 reference_field_world_;
  gtsam::Rot3 base_R_sensor_;

public:
  /**
   * @brief Constructor for MagFactorArm.
   * @param pose_key GTSAM key for the AUV pose.
   * @param measured_field The measured magnetic field vector (sensor frame).
   * @param reference_field The reference magnetic field vector (world frame).
   * @param base_R_sensor The static rotation from base to sensor.
   * @param noise_model The noise model for the measurement (Dimension must be 3).
   */
  MagFactorArm(
    gtsam::Key pose_key, const gtsam::Point3 & measured_field,
    const gtsam::Point3 & reference_field,
    const gtsam::Rot3 & base_R_sensor, const gtsam::SharedNoiseModel & noise_model)
  : NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
    measured_field_sensor_(measured_field),
    reference_field_world_(reference_field),
    base_R_sensor_(base_R_sensor)
  {
  }

  /**
   * @brief Evaluates the error and Jacobians for the factor.
   * @param pose The AUV pose estimate.
   * @param H Optional Jacobian matrix.
   * @return The 3D error vector (predicted - measured) in the sensor frame.
   */
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    // Predict the magnetic field in the body and sensor frames
    gtsam::Matrix33 H_unrotate_body;
    gtsam::Point3 predicted_field_body =
      pose.rotation().unrotate(reference_field_world_, H ? &H_unrotate_body : 0);
    gtsam::Point3 predicted_field_sensor = base_R_sensor_.unrotate(predicted_field_body);

    // 3D magnetic field residual
    gtsam::Vector3 error = predicted_field_sensor - measured_field_sensor_;

    if (H) {
      // Jacobian with respect to pose (3x6)
      H->setZero(3, 6);
      H->block<3, 3>(0, 0) = base_R_sensor_.inverse().matrix() * H_unrotate_body;
    }

    return error;
  }
};

}  // namespace coug_fgo::factors
