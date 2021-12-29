#ifndef MEKF_H_
#define MEKF_H_


#include "buffer.hpp"
#include "common.h"



namespace mekf{

    class MEKF{

        public:

            static constexpr int k_num_states_ = 15;

            MEKF();

            void updateCamPose(cameraPoseSample cam_pose_sample);

            void run_mekf(imuSample imu_sample);

        private:

            // state
            state state_;
            
            // FIFO buffers
            Buffer<cameraPoseSample> camPoseBuffer_;
            Buffer<imuSample> imuBuffer_;

            // FIFO buffer lengths 
            const int cam_buffer_length_ {9};  // TODO: check lengths
            const int imu_buffer_length_ {15}; // TODO: check lengths

            // filter initalization
            bool initializeFilter();
            bool filter_initialised_; // true when initialized 

            // Covariance
            Eigen::Matrix<double, k_num_states_, k_num_states_> P_prd; // System covariance matrix

            // process noise weights: v, acc_bias, w, ars_bias, TODO: static? 12 x 12
            Eigen::Matrix<double, 12, 12> Q_d;

            // measurement noise - position aiding + compass 
            Eigen::Matrix<double, 7, 7> R_d;
    };

}


#endif /* defined(MEKF_H_) */