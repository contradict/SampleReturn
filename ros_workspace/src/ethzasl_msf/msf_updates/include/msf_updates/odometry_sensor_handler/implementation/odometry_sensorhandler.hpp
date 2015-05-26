/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef ODOMETRY_SENSORHANDLER_HPP_
#define ODOMETRY_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>

namespace msf_odometry_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
OdometrySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::OdometrySensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-5),
      n_zy_(1e-5),
      n_zv_(1e-4),
      n_zw_(1e-4),
      delay_(0) {
  ros::NodeHandle pnh("~/" + parameternamespace);
  pnh.param("odometry_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("odometry_absolute_measurements", provides_absolute_measurements_,
            false);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Odometry sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Odometry sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Odometry sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Odometry sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subOdometry_ =
      nh.subscribe<nav_msgs::Odometry>
  ("odometry_input", 20, &OdometrySensorHandler::MeasurementCallback, this);

  z_p_.setZero();

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void OdometrySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_zp, double n_zy, double n_zv, double n_zw) {
  n_zp_ = n_zp;
  n_zy_ = n_zy;
  n_zv_ = n_zv;
  n_zw_ = n_zw;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void OdometrySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void OdometrySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessOdometryMeasurement(
    const nav_msgs::OdometryConstPtr& msg) {
  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->pose.covariance[0] == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().odometry_fixed_p_ib) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ib;
    }
    if (mngr->Getcfg().odometry_fixed_q_ib) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::q_ib;
    }
    if (mngr->Getcfg().odometry_fixed_p_d) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_d;
    }
    if (mngr->Getcfg().odometry_fixed_yaw_d) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::yaw_d;
    }
  }

  shared_ptr < MEASUREMENT_TYPE
      > meas(
          new MEASUREMENT_TYPE(n_zp_, n_zy_, n_zv_, n_zw_, use_fixed_covariance_,
                               provides_absolute_measurements_, this->sensorID,
                               fixedstates));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void OdometrySensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const nav_msgs::OdometryConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subOdometry_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** odometry sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subOdometry_.getTopic()
          << " ***");
  ProcessOdometryMeasurement(msg);
}

}  // namespace msf_odometry_sensor
#endif  // ODOMETRY_SENSORHANDLER_HPP_
