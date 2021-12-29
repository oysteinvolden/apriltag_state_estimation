#include <mekf/mekf.h>



namespace mekf{

    MEKF::MEKF(){
      
        // initialize INS
        state_.quat_nominal = quat(1,0,0,0);
        state_.vel = vec3(0,0,0);
        state_.pos = vec3(0,0,0);
        state_.gyro_bias = vec3(0,0,0);
        state_.accel_bias = vec3(0,0,0);
        
         
        // allocate imu buffer
        imuBuffer_.allocate(imu_buffer_length_);
        for (int i = 0; i < imu_buffer_length_; i++){
            imuSample imu_sample_init = {};
            imuBuffer_.push(imu_sample_init);
        }

        
        // allocate camera pose buffer
        camPoseBuffer_.allocate(cam_buffer_length_);
        for (int i = 0; i < cam_buffer_length_; i++) {
            cameraPoseSample camera_pose_sample_init = {};
            camPoseBuffer_.push(camera_pose_sample_init);
        }
        
        // set to false, so we can initialize
        filter_initialised_ = false;

    }


    bool MEKF::initializeFilter(){

        // initialize covariance
        P_prd.setIdentity(k_num_states_,k_num_states_);

        // TODO: tune Q_d and R_d later

        // initialize process weights
        Q_d.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001;

        // initialize measurement weights
        R_d.diagonal() << 1, 1, 1, 1, 1, 1, 0.01;

        return true;
    }


    void MEKF::updateCamPose(cameraPoseSample cam_pose_sample){

        // push pose to buffer
        // TODO: check if struct is nonzero
        camPoseBuffer_.push(cam_pose_sample);

    }



    void MEKF::run_mekf(imuSample imu_sample){

        // push to buffer
        // TODO: check if struct is nonzero
        imuBuffer_.push(imu_sample);

        // check if filter is initialized
        if (!filter_initialised_) {
            filter_initialised_ = initializeFilter();
            if (!filter_initialised_) {
                return;
            }
        }

        
        // input: x_ins,P_prd,mu,h,Qd,Rd,f_imu,w_imu,y_psi
        // output: x_ins, P_prd

    }

}


