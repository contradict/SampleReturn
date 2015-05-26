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
#ifndef ODOMETRY_SENSOR_H_
#define ODOMETRY_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>
#include <nav_msgs/Odometry.h>

namespace msf_odometry_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class OdometrySensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 6, 1> z_p_;  ///< Odometry measurement.
  double n_zp_;  ///< Position measurement noise.
  double n_zy_;  ///< Yaw measurement noise.
  double n_zv_;  ///< Velocity measurement noise.
  double n_zw_;  ///< Angular rate measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of
                       //the measurement provided by this sensor.

  ros::Subscriber subOdometry_;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void ProcessOdometryMeasurement(
      const nav_msgs::OdometryConstPtr& msg);
  void MeasurementCallback(const nav_msgs::OdometryConstPtr& msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  OdometrySensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 6, 1> GetOdometryMeasurement() {
    return z_p_;
  }
  // Setters for configure values.
  void SetNoises(double n_zp, double n_zy, double n_zv, double n_zw);
  void SetDelay(double delay);
};
}  // namespace msf_odometry_sensor

#include "implementation/odometry_sensorhandler.hpp"

#endif  // ODOMETRY_SENSOR_H_
