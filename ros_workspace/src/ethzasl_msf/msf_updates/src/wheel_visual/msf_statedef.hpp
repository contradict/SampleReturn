/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MSF_STATEDEF_HPP_
#define MSF_STATEDEF_HPP_

#include <Eigen/Dense>
#include <msf_core/msf_fwds.h>
#define FUSION_MAX_VECTOR_SIZE 15
#include <boost/fusion/container.hpp>
namespace msf_updates {
/*
 * This file contains the state definition of the EKF as defined for a given set
 * of sensors / states to estimate.
 */
enum StateDefinition {  //must not manually set the enum values!
  p,                    // position of imu in world coordinates
  v,                    // velocity of imu in world coordinates
  q,                    // rotate a vector in imu coordinates to world coordinates
  b_w,                  // gyro bias in imu coordinates
  b_a,                  // accel bias in imu coordinates
  L,                    // visual scale factor
  q_wv,                 // rotate a vector in vision frame to world frame
  p_wv,                 // position of vision frame in world cooridnates
  q_ic,                 // rotate a vector in camera frame to imu coordinates
  p_ic,                 // position of camera in imu cooridnates (backwards)
  yaw_d,                // yaw from world frame to odometry frame in odometry frame
  p_d,                 // position of world frame origin in odometry frame
  q_ib,                 // rotate vector in base link to imu
  p_ib,                 // position of base link in imu coordinates
};

namespace {
/***
 * Setup core state, then auxiliary state.
 */
typedef boost::fusion::vector<
    // States varying during propagation - must not change the ordering here for
    // now, CalcQ has the ordering hardcoded.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p,
        msf_core::CoreStateWithPropagation>,  ///< Translation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, v,
        msf_core::CoreStateWithPropagation>,  ///< Velocity of the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q,
        msf_core::CoreStateWithPropagation>,  ///< Rotation from the world frame to the IMU frame expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_w,
        msf_core::CoreStateWithoutPropagation>,  ///< Gyro biases.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, b_a,
        msf_core::CoreStateWithoutPropagation>,  ///< Acceleration biases.

    // States not varying during propagation.
    msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, L>,     ///< Visual scale.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_wv,  ///< Rotation from the world frame to the frame in which the pose is measured expressed in the world frame.
        msf_core::AuxiliaryNonTemporalDrifting>,  ///< Translation from the world frame to the frame in which the pose is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_wv>,  ///< Vision world position drift.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_ic>,  ///< Rotation from the IMU frame to the camera frame expressed in the IMU frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ic>,  ///< Translation from the IMU frame to the camera frame expressed in the IMU frame.

    msf_core::StateVar_T<Eigen::Matrix<double, 1, 1>, yaw_d,  ///< Rotation from the world frame to the frame in which the wheel odometry is measured expressed in the world frame.
        msf_core::AuxiliaryNonTemporalDrifting>,  ///< Translation from the world frame to the frame in which the wheel odometry is measured expressed in the world frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 2, 1>, p_d>,  ///< wheel odometry position drift.
    msf_core::StateVar_T<Eigen::Quaternion<double>, q_ib>,  ///< Rotation from the IMU frame to the base link frame expressed in the IMU frame.
    msf_core::StateVar_T<Eigen::Matrix<double, 3, 1>, p_ib> ///< Translation from the IMU frame to the base link frame expressed in the IMU frame.
> fullState_T;
}
///< The state we want to use in this EKF.
typedef msf_core::GenericState_T<fullState_T, StateDefinition> EKFState;
typedef shared_ptr<EKFState> EKFStatePtr;
typedef shared_ptr<const EKFState> EKFStateConstPtr;
}
#include <msf_updates/static_ordering_assertions.h> //DO NOT REMOVE THIS
#endif  // MSF_STATEDEF_HPP_
