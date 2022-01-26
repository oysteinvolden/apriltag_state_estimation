#ifndef MEKF_H_
#define MEKF_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <iostream>
#include <math.h>

#include "buffer.hpp"
#include "common.h"



namespace mekf{

    class MEKF{

        public:

            static constexpr int k_num_states_ = 15; // not 16 since we parametrize unit quaternion with three parameters (error state)

            MEKF();

            void updateCamPose(const vec3& cam_pos, const quat& cam_quat, uint64_t time_usec);

            void run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, scalar_t dt);

            quat getQuat();
            vec3 getPosition();
            vec3 getVelocity();
            uint64_t getImuTime(); 

        private:

            
            // state
            state state_;
            
            // FIFO buffers
            Buffer<cameraPoseSample> camPoseBuffer_;
            Buffer<imuSample> imuBuffer_;

            // FIFO buffer lengths 
            const int cam_buffer_length_ {9};  // TODO: check lengths
            const int imu_buffer_length_ {15}; // TODO: check lengths

            // new samples
            imuSample imu_sample_new_ {};  // imu sample capturing the newest imu data
            cameraPoseSample cam_pose_sample_new_ {}; // cam sample capturing the newest vision data

            // delayed samples
            imuSample imu_sample_delayed_ {};	// captures the imu sample on the delayed time horizon
            cameraPoseSample cam_pose_delayed_ {}; // captures the cam pose sample on the delayed time horizon

            // flags on received updates
            bool cam_pose_ready_ = false;

            // timing
            uint64_t time_last_cam_pose_ {0};
            uint64_t time_last_imu_ {0};
            uint64_t current_cam_pose_time = {0}; // used to check if fresh cam pose is available

            // sampling constants
            const double f_s  = 100; // sampling frequency [Hz]
            const double h = 1/f_s; // sampling time [s]

            // sensors delay
            //scalar_t cam_pose_delay_ = {100.0d}; // vision measurement delay relative to the IMU (mSec) - TODO: neccessary? 

            // filter initalization
            bool initializeFilter();
            bool filter_initialised_; // true when initialized 


            // gravity constants
            const double mu = 63.2623 * (M_PI/180); // lattitude (Trondheim brattøra), TODO: make more configurable?
            double g;
            vec3 g_n;
            double gravity(double lattitude);

            // Bias time constants (user specified)
            const double T_acc = 1000; 
            const double T_gyro = 1000; 

            // Covariance and predictor
            Eigen::Matrix<double, k_num_states_, k_num_states_> P_prd, P_hat; 

            // TODO: add k_num_states wherever possible for matrices

            // process noise weights: v, acc_bias, w, ars_bias
            Eigen::Matrix<double, 12, 12> Qd;

            // measurement noise - position aiding + compass 
            Eigen::Matrix<double, 7, 7> Rd;

            // constant matrices
            Eigen::Matrix<double, 3, 3> O3, I3;
            Eigen::Matrix<double, 15, 15> I15;
            Eigen::Matrix<double, 1, 3> O_13;
            Eigen::Matrix<double, 1, 9> O_19;

            // reference vector
            vec3 v01;

            // Rotation matrix
            Eigen::Matrix<double,3,3> R;

            // Discrete-time KF matrices
            Eigen::Matrix<double, 15, 15> A, Ad;
            Eigen::Matrix<double, 7, 15> Cd;
            Eigen::Matrix<double, 15, 12> Ed;

            // kalman gain
            Eigen::Matrix<double, 15, 7> K;
            Eigen::Matrix<double, 15, 15> IKC;

            // smallest signed angle
            double ssa(double angle);

            // epsilon matrix
            Eigen::Matrix<double, 7, 1> eps;

            // estimated error state
            Eigen::Matrix<double, 15, 1> delta_x_hat;

    };

}


#endif /* defined(MEKF_H_) */