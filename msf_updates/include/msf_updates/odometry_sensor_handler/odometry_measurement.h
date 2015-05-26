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
#ifndef ODOMETRY_MEASUREMENT_HPP_
#define ODOMETRY_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>
#include <nav_msgs/Odometry.h>

namespace msf_updates {
    namespace odometry_measurement {
        enum {
            nMeasurements = 3+3
        };

        /**
         * \brief A measurement as provided by a wheel odometry sensor
         * (x, y, yaw, vx, vy, omega_z)
         */
        typedef msf_core::MSF_Measurement<
            nav_msgs::Odometry,
            Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> OdometryMeasurementBase;
        struct OdometryMeasurement : public OdometryMeasurementBase {
            private:
                typedef OdometryMeasurementBase Measurement_t;
                typedef Measurement_t::Measurement_ptr measptr_t;

                virtual void MakeFromSensorReadingImpl(measptr_t msg) {

                    Eigen::Matrix<double, nMeasurements,
                        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
                    Eigen::Matrix<double, nMeasurements, 1> r_old;

                    H_old.setZero();

                    // Get measurement.
                    z_p_ = (Eigen::Matrix<double, nMeasurements, 1>() <<
                            msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.orientation.z,
                            msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z
                            ).finished();

                    if (fixed_covariance_)  //  take fix covariance from reconfigure GUI
                    {

                        const double s_zp = n_zp_ * n_zp_;
                        const double s_zy = n_zy_ * n_zy_;
                        const double s_zv = n_zv_ * n_zv_;
                        const double s_zw = n_zw_ * n_zw_;
                        R_ = (Eigen::Matrix<double, nMeasurements, 1>() <<
                                s_zp, s_zp, s_zy,
                                s_zv, s_zv, s_zw)
                            .finished().asDiagonal();

                    } else {  // Tke covariance from sensor.

                        Eigen::Matrix<double, 6, 6> meas_cov(&msg->pose.covariance[0]);

                        R_.block<2, 2>(0, 0) = meas_cov.block<2, 2>(0,0);
                        R_.block<1, 1>(2, 2) = meas_cov.block<1, 1>(5,5);
                        R_.block<2, 2>(3, 3) = meas_cov.block<2, 2>(0,0);
                        R_.block<1, 1>(5, 5) = meas_cov.block<1, 1>(5,5);

                        if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
                            if (R_.block<3, 3>(0, 0).determinant() < -0.01)
                                MSF_WARN_STREAM_THROTTLE(
                                        60, "The covariance matrix you provided for "
                                        "the position sensor is not positive definite");
                        }
                    }
                }
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                    Eigen::Matrix<double, nMeasurements, 1> z_p_;  /// Position measurement.
                double n_zp_;  /// Position measurement noise.
                double n_zy_;  /// Yaw measurement noise.
                double n_zv_;  /// Velocity measurement noise.
                double n_zw_;  /// Angular rate measurement noise.

                bool fixed_covariance_;
                int fixedstates_;

                typedef msf_updates::EKFState EKFState_T;
                typedef EKFState_T::StateSequence_T StateSequence_T;
                typedef EKFState_T::StateDefinition_T StateDefinition_T;
                virtual ~OdometryMeasurement() {
                }
                OdometryMeasurement(double n_zp, double n_zy, double n_zv, double n_zw,
                        bool fixed_covariance,
                        bool isabsoluteMeasurement, int sensorID, int fixedstates)
                    : OdometryMeasurementBase(isabsoluteMeasurement, sensorID),
                    n_zp_(n_zp),
                    n_zy_(n_zy),
                    n_zv_(n_zv),
                    n_zw_(n_zw),
                    fixed_covariance_(fixed_covariance),
                    fixedstates_(fixedstates) {
                    }
                virtual std::string Type() {
                    return "odometry";
                }

                virtual void CalculateH(
                        shared_ptr<EKFState_T> state_in,
                        Eigen::Matrix<double, nMeasurements,
                        msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
                    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

                    H.setZero();

                    Eigen::Matrix<double, 3, 1> p_iw = state.Get<StateDefinition_T::p>();

                    Eigen::Matrix<double, 3, 1> v_iw = state.Get<StateDefinition_T::v>();

                    Eigen::Quaterniond q_iw = state.Get<StateDefinition_T::q>();
                    Eigen::Matrix<double, 3, 3> C_iw = q_iw.toRotationMatrix();

                    Eigen::Matrix<double, 3, 1> p_ib = state.Get<StateDefinition_T::p_ib>();

                    Eigen::Quaterniond q_ib = state.Get<StateDefinition_T::q_ib>();
                    Eigen::Matrix<double, 3, 3> C_ib = q_ib.toRotationMatrix();
                    Eigen::Matrix<double, 3, 3> C_bi = C_ib.transpose();

                    double yaw_d = state.Get<StateDefinition_T::yaw_d>()(0);
                    Eigen::Quaterniond q_d = Eigen::Quaterniond(sqrt(1.0-yaw_d*yaw_d/4.0), 0, 0, yaw_d/2.0);
                    Eigen::Matrix<double, 3, 3> C_d = q_d.toRotationMatrix();

                    Eigen::Matrix<double, 2, 1> p_d = state.Get<StateDefinition_T::p_d>();

                    Eigen::Matrix<double, 3, 3> omega_sk = Skew(state.w_m);

                    // decompose to yaw about z
                    Eigen::Quaterniond q_bw = q_iw*q_ib;
                    Eigen::Quaterniond q_bw_y = Eigen::Quaterniond(q_bw.w(), 0, 0, q_bw.z()).normalized();
                    Eigen::Matrix<double, 3, 3> C_bw_y = q_bw_y.toRotationMatrix();
                    // and pitch/roll about vector in xy plane
                    Eigen::Quaterniond q_bw_pr = q_bw*q_bw_y.conjugate();

                    // construct odometry->world rotation
                    Eigen::Quaterniond q_wo = (q_bw_pr*q_d).conjugate();
                    Eigen::Matrix<double, 3, 3> C_wo = q_wo.toRotationMatrix();
                    // C_wo = C_d.transpose()*C_bw_y*C_bi*C_iw.transpose()


                    // Get indices of states in error vector.
                    enum {
                        kIdxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p>::value,
                        kIdxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::v>::value,
                        kIdxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::q>::value,

                        kIdxstartcorr_yawd = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::yaw_d>::value,
                        kIdxstartcorr_pd = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p_d>::value,
                        kIdxstartcorr_qib = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::q_ib>::value,
                        kIdxstartcorr_pib = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
                        StateDefinition_T::p_ib>::value,
                    };

                    // Read the fixed states flags.
                    bool calibposfix = (fixedstates_ & 1 << StateDefinition_T::p_ib);
                    bool calibattfix = (fixedstates_ & 1 << StateDefinition_T::q_ib);
                    bool driftdattfix = (fixedstates_ & 1 << StateDefinition_T::yaw_d);
                    bool driftdposfix = (fixedstates_ & 1 << StateDefinition_T::p_d);

                    // Set crosscov to zero for fixed states.
                    if (calibposfix)
                        state_in->ClearCrossCov<StateDefinition_T::p_ib>();
                    if (calibattfix)
                        state_in->ClearCrossCov<StateDefinition_T::q_ib>();
                    if (driftdattfix)
                        state_in->ClearCrossCov<StateDefinition_T::yaw_d>();
                    if (driftdposfix)
                        state_in->ClearCrossCov<StateDefinition_T::p_d>();

                    // Construct H matrix.
                    // Position:
                    H.block<2, 3>(0, kIdxstartcorr_p) = C_wo.block<2,3>(0,0);  // p

                    auto position = [&q_iw, &q_ib, &p_ib, &p_iw, &q_d, &p_d] (
                            Eigen::Quaterniond dq_iw=Eigen::Quaterniond::Identity(),
                            Eigen::Quaterniond dq_ib=Eigen::Quaterniond::Identity())
                        -> Eigen::Matrix<double, 2,1>
                    {
                        Eigen::Quaterniond q_bw = dq_iw*q_iw*dq_ib*q_ib;
                        Eigen::Quaterniond q_bw_y = Eigen::Quaterniond(q_bw.w(), 0, 0, q_bw.z()).normalized();
                        // and pitch/roll about vector in xy plane
                        Eigen::Quaterniond q_bw_pr = q_bw*q_bw_y.conjugate();
                        // construct odometry->world rotation
                        Eigen::Quaterniond q_wo = (q_bw_pr*q_d).conjugate();

                        return ((q_wo*(p_iw + dq_iw*q_iw*p_ib)).block<2,1>(0,0) + p_d);
                    };
                    double eps = 0.001;
                    Eigen::Quaterniond dq_iw;
                    dq_iw.setIdentity();
                    dq_iw.x() = eps/2.;
                    H.block<2,1>(0, kIdxstartcorr_q) = (position(dq_iw)-position())/eps;
                    dq_iw.setIdentity();
                    dq_iw.y() = eps/2.;
                    H.block<2,1>(0, kIdxstartcorr_q+1) = (position(dq_iw)-position())/eps;
                    dq_iw.setIdentity();
                    dq_iw.z() = eps/2.;
                    H.block<2,1>(0, kIdxstartcorr_q+2) = (position(dq_iw)-position())/eps;

                    H.block<2, 3>(0, kIdxstartcorr_pib) =
                        calibposfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() :
                        (C_d.transpose()*C_bw_y*C_ib.transpose()).block<2,3>(0,0).eval();  //p_ib

                    if(calibattfix)
                    {
                        H.block<2, 3>(0, kIdxstartcorr_qib) =  Eigen::Matrix<double, 2, 3>::Zero();  // q_ib
                    }
                    else
                    {
                        Eigen::Quaterniond dq_ib;
                        dq_ib.setIdentity();
                        dq_ib.x() = eps/2.;
                        H.block<2,1>(0, kIdxstartcorr_qib) = (position(Eigen::Quaterniond::Identity(), dq_ib)-position())/eps;
                        dq_ib.setIdentity();
                        dq_ib.y() = eps/2.;
                        H.block<2,1>(0, kIdxstartcorr_qib+1) = (position(Eigen::Quaterniond::Identity(), dq_ib)-position())/eps;
                        dq_ib.setIdentity();
                        dq_ib.z() = eps/2.;
                        H.block<2,1>(0, kIdxstartcorr_qib+2) = (position(Eigen::Quaterniond::Identity(), dq_ib)-position())/eps;
                    }

                    H.block<2, 2>(0, kIdxstartcorr_pd) =
                        driftdposfix ?
                        (Eigen::Matrix<double, 2, 2>::Zero().eval()):
                        (Eigen::Matrix<double, 3, 3>::Identity().block<2, 2>(0,0).eval());  //p_d

                    H.block<2, 1>(0, kIdxstartcorr_yawd) =
                        driftdattfix ?
                        Eigen::Matrix<double, 2, 1>::Zero() :
                        (-C_d.transpose()*Skew(C_bw_y*C_bi*(C_iw.transpose()*p_iw+p_ib))).block<2,1>(0,2).eval();  // yaw_d

                    // Yaw.

                    auto orientation = [&q_iw, &q_ib, &q_d] (
                            Eigen::Quaterniond dq_iw=Eigen::Quaterniond::Identity(),
                            Eigen::Quaterniond dq_ib=Eigen::Quaterniond::Identity())
                        ->Eigen::Quaterniond
                    {
                        Eigen::Quaterniond q_bw = dq_iw*q_iw*dq_ib*q_ib;
                        Eigen::Quaterniond q_bw_y = Eigen::Quaterniond(q_bw.w(), 0, 0, q_bw.z()).normalized();
                        // and pitch/roll about vector in xy plane
                        Eigen::Quaterniond q_bw_pr = q_bw*q_bw_y.conjugate();
                        // construct odometry->world rotation
                        Eigen::Quaterniond q_wo = (q_bw_pr*q_d).conjugate();

                        return q_wo*dq_iw*q_iw*dq_ib*q_ib;
                    };
                    dq_iw.setIdentity();
                    dq_iw.x() = eps/2.;
                    H(2, kIdxstartcorr_q) = (orientation(dq_iw).conjugate()*orientation()).z()/eps*2;
                    dq_iw.setIdentity();
                    dq_iw.y() = eps/2.;
                    H(2, kIdxstartcorr_q+1) = (orientation(dq_iw).conjugate()*orientation()).z()/eps*2;
                    dq_iw.setIdentity();
                    dq_iw.z() = eps/2;
                    H(2, kIdxstartcorr_q+2) = (orientation(dq_iw).conjugate()*orientation()).z()/eps*2;

                    H.block<1, 3>(2, kIdxstartcorr_yawd) =
                        driftdattfix ?
                        Eigen::Matrix<double, 1, 3>::Zero() :
                        (-Eigen::Matrix<double, 3, 3>::Identity()).block<1,3>(2,0).eval();  // yaw_d

                    if(calibattfix)
                    {
                        H.block<1, 3>(2, kIdxstartcorr_qib) = Eigen::Matrix<double, 1, 3>::Zero();
                    }
                    else
                    {
                        Eigen::Quaterniond dq_ib;
                        dq_ib.setIdentity();
                        dq_ib.x() = eps/2.;
                        H(2, kIdxstartcorr_qib) =  (orientation(Eigen::Quaterniond::Identity(), dq_ib).conjugate()*orientation()).z()/eps*2;
                        dq_ib.setIdentity();
                        dq_ib.y() = eps/2.;
                        H(2, kIdxstartcorr_qib+1) =  (orientation(Eigen::Quaterniond::Identity(), dq_ib).conjugate()*orientation()).z()/eps*2;
                        dq_ib.setIdentity();
                        dq_ib.z() = eps/2.;
                        H(2, kIdxstartcorr_qib+2) =  (orientation(Eigen::Quaterniond::Identity(), dq_ib).conjugate()*orientation()).z()/eps*2;
                    }

                    // Velocity
                    H.block<2, 3>(3, kIdxstartcorr_v) = (C_bi * C_iw.transpose()).block<2,3>(0,0);  // v

                    H.block<2, 3>(3, kIdxstartcorr_q) =
                        (-C_bi*Skew(C_iw.transpose() * v_iw)).block<2,3>(0,0);  // q

                    H.block<2, 3>(3, kIdxstartcorr_pib) = 
                        calibposfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() : 
                        (C_bi * omega_sk).block<2,3>(0,0).eval(); // p_ib

                    H.block<2, 3>(3, kIdxstartcorr_qib) = 
                        calibattfix ?
                        Eigen::Matrix<double, 2, 3>::Zero() : 
                        (-C_bi*Skew((C_iw.transpose() * v_iw + omega_sk * p_ib))).block<2,3>(0,0).eval(); // q_ib

                    // Angular Rate
                    H.block<1,3>(5, kIdxstartcorr_qib) =
                        calibattfix ?
                        Eigen::Matrix<double, 1, 3>::Zero() : 
                        (-C_bi*Skew(state.w_m)).block<1,3>(2,0).eval(); // q_ib

                    //MSF_INFO_STREAM("p_iw:" << p_iw.transpose() << " v_iw:" << v_iw.transpose() << " q_iw:" << STREAMQUAT(q_iw));
                    //MSF_INFO_STREAM("p_ib:" << p_ib.transpose() << " q_ib:" << STREAMQUAT(q_ib));
                    //MSF_INFO_STREAM("p_d:" << p_d.transpose() << " yaw_d:" << yaw_d);
                    //MSF_INFO_STREAM("H[:,0:9]:\n" << H.leftCols(9));
                    //MSF_INFO_STREAM("H[:,9:28]:\n" << H.middleCols<19>(9));
                    //MSF_INFO_STREAM("H[:,28:37]:\n" << H.middleCols<9>(28));
                }

                /**
                 * The method called by the msf_core to apply the measurement represented by this object.
                 */
                virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                        msf_core::MSF_Core<EKFState_T>& core) {


                    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
                        // or is is just relative to the last measurement.
                        // Get a const ref, so we can read core states
                        const EKFState_T& state = *state_nonconst_new;

                        Eigen::Matrix<double, 3, 3> omega_sk = Skew(state.w_m);

                        // init variables
                        Eigen::Matrix<double, nMeasurements,
                            msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
                        Eigen::Matrix<double, nMeasurements, 1> r_old;

                        CalculateH(state_nonconst_new, H_new);

                        Eigen::Matrix<double, 3, 1> p_iw = state.Get<StateDefinition_T::p>();
                        Eigen::Matrix<double, 3, 1> v_iw = state.Get<StateDefinition_T::v>();

                        Eigen::Quaterniond q_ib = state.Get<StateDefinition_T::q_ib>();

                        Eigen::Quaterniond q_iw = state.Get<StateDefinition_T::q>();
                        // decompose to yaw about z
                        Eigen::Quaterniond q_bw = q_iw*q_ib;
                        Eigen::Quaterniond q_bw_y = Eigen::Quaterniond(q_bw.w(), 0, 0, q_bw.z()).normalized();
                        // and pitch/roll about vector in xy plane
                        Eigen::Quaterniond q_bw_pr = q_bw*q_bw_y.conjugate();

                        Eigen::Matrix<double, 3, 1> p_ib = state.Get<StateDefinition_T::p_ib>();

                        double yaw_d = state.Get<StateDefinition_T::yaw_d>()(0);
                        Eigen::Quaterniond q_d = Eigen::Quaterniond(sqrt(1.0-yaw_d*yaw_d/4.0), 0, 0, yaw_d/2.0);
                        // construct odometry->world rotation
                        Eigen::Quaterniond q_wo = (q_bw_pr*q_d).conjugate();

                        Eigen::Matrix<double, 2, 1> p_d = state.Get<StateDefinition_T::p_d>();

                        // Construct residuals.
                        // Position.
                        r_old.block<2, 1>(0, 0) = z_p_.block<2,1>(0,0)
                                - ((q_wo*(p_iw + q_iw*p_ib)).block<2,1>(0,0) + p_d);

                        //MSF_INFO_STREAM("position measured: " << (z_p_.block<2,1>(0,0).transpose()));
                        //MSF_INFO_STREAM("        predicted: " << (((q_wo*(p_iw + q_iw*p_ib)).block<2,1>(0,0) + p_d).transpose()));

                        // Yaw.
                        Eigen::Quaternion<double> q_err;
                        Eigen::Quaternion<double> z_q(sqrt(1.0-z_p_(2)*z_p_(2)), 0.0, 0.0, z_p_(2));
                        q_err = (q_wo * q_iw * q_ib).conjugate() * z_q;
                        // Odometry world yaw drift - virtual measurement to fix yaw to this world
                        // q_err = q_d;
                        r_old.block<1, 1>(2, 0) = (q_err.vec() / q_err.w() * 2).block<1,1>(2,0);

                        //MSF_INFO_STREAM("yaw measured: " << STREAMQUAT(z_q));
                        //MSF_INFO_STREAM("   predicted: " << STREAMQUAT((q_wo * q_iw * q_ib)));

                        // r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
                        //     / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

                        // velocity
                        r_old.block<2,1>(3,0) = z_p_.block<2,1>(3,0) -
                            (q_ib.conjugate()*(q_iw.conjugate() * v_iw + omega_sk*p_ib)).block<2,1>(0,0);

                        //MSF_INFO_STREAM("velocity measured: " << (z_p_.block<2,1>(3,0).transpose()));
                        //MSF_INFO_STREAM("        predicted: " <<
                        //    ((q_ib.conjugate()*(q_iw.conjugate() * v_iw + omega_sk*p_ib)).block<2,1>(0,0).transpose())
                        //    );

                        // angular rate z
                        r_old.block<1,1>(5,0) = z_p_.block<1,1>(5,0) - (q_ib.conjugate()*state.w_m).block<1,1>(2,0);

                        //MSF_INFO_STREAM("angular rate measured: " << (z_p_.block<1,1>(5,0)));
                        //MSF_INFO_STREAM("            predicted: " << ((q_ib.conjugate()*state.w_m).block<1,1>(2,0)));

                        //MSF_INFO_STREAM("r_old: " << r_old.transpose());

                        if (!CheckForNumeric(r_old, "r_old")) {
                            MSF_ERROR_STREAM("r_old: "<<r_old);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(H_new, "H_old")) {
                            MSF_ERROR_STREAM("H_old: "<<H_new);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(R_, "R_")) {
                            MSF_ERROR_STREAM("R_: "<<R_);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
                        }

                        // Call update step in base class.
                        this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                R_);
                    } else {
                        // Init variables: Get previous measurement.
                        shared_ptr < msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base =
                            core.GetPreviousMeasurement(this->time, this->sensorID_);

                        if (prevmeas_base->time == msf_core::constants::INVALID_TIME) {
                            MSF_WARN_STREAM(
                                    "The previous measurement is invalid. Could not apply measurement! " "time:"<<this->time<<" sensorID: "<<this->sensorID_);
                            return;
                        }

                        // Make this a pose measurement.
                        shared_ptr<OdometryMeasurement> prevmeas = dynamic_pointer_cast
                            < OdometryMeasurement > (prevmeas_base);
                        if (!prevmeas) {
                            MSF_WARN_STREAM(
                                    "The dynamic cast of the previous measurement has failed. "
                                    "Could not apply measurement");
                            return;
                        }

                        // Get state at previous measurement.
                        shared_ptr<EKFState_T> state_nonconst_old = core.GetClosestState(
                                prevmeas->time);

                        if (state_nonconst_old->time == msf_core::constants::INVALID_TIME) {
                            MSF_WARN_STREAM(
                                    "The state at the previous measurement is invalid. Could "
                                    "not apply measurement");
                            return;
                        }

                        // Get a const ref, so we can read core states.
                        const EKFState_T& state_new = *state_nonconst_new;
                        const EKFState_T& state_old = *state_nonconst_old;

                        Eigen::Matrix<double, nMeasurements,
                            msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new,
                            H_old;
                        Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

                        CalculateH(state_nonconst_old, H_old);

                        H_old *= -1;

                        CalculateH(state_nonconst_new, H_new);

                        Eigen::Matrix<double, 3, 3> omega_sk_new = Skew(state_new.w_m);
                        Eigen::Matrix<double, 3, 3> omega_sk_old = Skew(state_old.w_m);

                        //TODO (slynen): check that both measurements have the same states fixed!
                        Eigen::Matrix<double, 3, 1> p_iw_old = state_old.Get<StateDefinition_T::p>();
                        Eigen::Matrix<double, 3, 1> v_iw_old = state_old.Get<StateDefinition_T::v>();

                        Eigen::Quaterniond q_iw_old = state_old.Get<StateDefinition_T::q>();

                        Eigen::Matrix<double, 3, 1> p_ib_old = state_old.Get<StateDefinition_T::p_ib>();

                        Eigen::Quaterniond q_ib_old = state_old.Get<StateDefinition_T::q_ib>();

                        double yaw_d_old = state_old.Get<StateDefinition_T::yaw_d>()(0);
                        Eigen::Quaterniond q_d_old = Eigen::Quaterniond(sqrt(1.0-yaw_d_old*yaw_d_old/4.0), 0, 0, yaw_d_old/2.0);

                        Eigen::Matrix<double, 2, 1> p_d_old = state_old.Get<StateDefinition_T::p_d>();

                        // decompose to yaw about z
                        Eigen::Quaterniond q_bw_old = q_iw_old*q_ib_old;
                        Eigen::Quaterniond q_bw_y_old = Eigen::Quaterniond(q_bw_old.w(), 0, 0, q_bw_old.z()).normalized();
                        // and pitch/roll about vector in xy plane
                        Eigen::Quaterniond q_bw_pr_old = q_bw_old*q_bw_y_old.conjugate();

                        // construct odometry->world rotation
                        Eigen::Quaterniond q_wo_old = (q_bw_pr_old*q_d_old).conjugate();


                        Eigen::Matrix<double, 3, 1> p_iw_new = state_new.Get<StateDefinition_T::p>();
                        Eigen::Matrix<double, 3, 1> v_iw_new = state_new.Get<StateDefinition_T::v>();

                        Eigen::Quaterniond q_iw_new = state_new.Get<StateDefinition_T::q>();

                        Eigen::Matrix<double, 3, 1> p_ib_new = state_new.Get<StateDefinition_T::p_ib>();

                        Eigen::Quaterniond q_ib_new = state_new.Get<StateDefinition_T::q_ib>();

                        double yaw_d_new = state_new.Get<StateDefinition_T::yaw_d>()(0);
                        Eigen::Quaterniond q_d_new = Eigen::Quaterniond(sqrt(1.0-yaw_d_new*yaw_d_new/4.0), 0, 0, yaw_d_new/2.0);

                        Eigen::Matrix<double, 2, 1> p_d_new = state_new.Get<StateDefinition_T::p_d>();

                        // decompose to yaw about z
                        Eigen::Quaterniond q_bw_new = q_iw_new*q_ib_new;
                        Eigen::Quaterniond q_bw_y_new = Eigen::Quaterniond(q_bw_new.w(), 0, 0, q_bw_new.z()).normalized();
                        // and pitch/roll about vector in xy plane
                        Eigen::Quaterniond q_bw_pr_new = q_bw_new*q_bw_y_new.conjugate();

                        // construct odometry->world rotation
                        Eigen::Quaterniond q_wo_new = (q_bw_pr_new*q_d_new).conjugate();

                        // Construct residuals.
                        // Position:
                        Eigen::Matrix<double, 2, 1> diffprobpos =
                            ((q_wo_new*(p_iw_new + q_iw_new*p_ib_new)).block<2,1>(0,0) + p_d_new)
                           -((q_wo_old*(p_iw_old + q_iw_old*p_ib_old)).block<2,1>(0,0) + p_d_old);

                        Eigen::Matrix<double, 2, 1> diffmeaspos =
                            z_p_.block<2,1>(0,0) - prevmeas->z_p_.block<2,1>(0,0);

                        r_new.block<2, 1>(0, 0) = diffmeaspos - diffprobpos;

                        // Yaw:
                        Eigen::Quaternion<double> diffprobatt =
                            (q_wo_new * q_iw_new * q_ib_new).conjugate() *
                            (q_wo_old * q_iw_old * q_ib_old);

                        Eigen::Quaterniond z_q_(sqrt(1.0-z_p_(2,0)*z_p_(2,0)), 0, 0, z_p_(2,0));
                        Eigen::Quaterniond z_q_prev(sqrt(1.0-prevmeas->z_p_(2,0)*prevmeas->z_p_(2,0)), 0, 0, prevmeas->z_p_(2,0));
                        Eigen::Quaterniond diffmeasatt = z_q_.conjugate() * z_q_prev;
                        Eigen::Quaterniond qerr = diffprobatt.conjugate() * diffmeasatt;
                        r_new(2,0) = qerr.z()/qerr.w() * 2;

                        // Velocity:
                        Eigen::Matrix<double, 2, 1> diffprobvel =
                            (q_ib_new.conjugate() * (q_iw_new.conjugate() * v_iw_new + omega_sk_new*p_ib_new)).block<2,1>(0,0)
                           -(q_ib_old.conjugate() * (q_iw_old.conjugate() * v_iw_old + omega_sk_old*p_ib_old)).block<2,1>(0,0);

                        Eigen::Matrix<double, 2, 1> diffmeasvel = z_p_.block<2,1>(3,0) - prevmeas->z_p_.block<2,1>(3,0);
                        r_new.block<2,1>(3,0) = diffmeasvel - diffprobvel;

                        // angular rate
                        Eigen::Matrix<double, 1, 1> diffprobomega = (q_ib_new.conjugate()*state_new.w_m - q_ib_old.conjugate()*state_old.w_m).block<1,1>(2,0);
                        Eigen::Matrix<double, 1, 1> diffmeasomega = z_p_.block<1,1>(5,0) - prevmeas->z_p_.block<1,1>(5,0);
                        r_old.block<1,1>(5,0) = diffmeasomega - diffprobomega;

                        if (!CheckForNumeric(r_old, "r_old")) {
                            MSF_ERROR_STREAM("r_old: "<<r_old);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(H_new, "H_old")) {
                            MSF_ERROR_STREAM("H_old: "<<H_new);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }
                        if (!CheckForNumeric(R_, "R_")) {
                            MSF_ERROR_STREAM("R_: "<<R_);
                            MSF_WARN_STREAM(
                                    "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
                        }

                        // Call update step in base class.
                        this->CalculateAndApplyCorrectionRelative(state_nonconst_old,
                                state_nonconst_new, core, H_old,
                                H_new, r_new, R_);

                    }
                }
        };
    }  // namespace odometry_measurement
}  // namespace msf_updates

#endif  // ODOMETRY_MEASUREMENT_HPP_
