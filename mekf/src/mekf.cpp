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
        Qd.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.001, 0.001, 0.001;

        // initialize measurement weights
        Rd.diagonal() << 1, 1, 1, 1, 1, 1, 0.01;

        return true;
    }


    void MEKF::updateCamPose(const vec3& cam_pos, const quat& cam_quat, uint64_t time_usec){

        // calculate the system time-stamp for the mid point of the integration period
        // copy required data
        cam_pose_sample_new_.posNED = cam_pos;
        cam_pose_sample_new_.quatNED = cam_quat;
        cam_pose_sample_new_.posErr = 0.05; // TODO: check later
        cam_pose_sample_new_.angErr = 0.05; // TODO: check later
        cam_pose_sample_new_.time_us = time_usec; // TODO: do we need to subtract camera delay here?

        // update last time
        time_last_cam_pose_ = time_usec;

        // push pose to buffer
        camPoseBuffer_.push(cam_pose_sample_new_); 

    }



    void MEKF::run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, scalar_t dt){


        // copy imu data
        imu_sample_new_.delta_ang = vec3(ang_vel.x(), ang_vel.y(), ang_vel.z()) * dt; // current delta angle  (rad)
        imu_sample_new_.delta_vel = vec3(lin_acc.x(), lin_acc.y(), lin_acc.z()) * dt; // current delta velocity (m/s)
        imu_sample_new_.delta_ang_dt = dt;
        imu_sample_new_.delta_vel_dt = dt;
        imu_sample_new_.time_us = time_us;
        time_last_imu_ = time_us; // update last time

        // push to buffer
        imuBuffer_.push(imu_sample_new_); 
  
        // get the oldest data from the buffer
        imu_sample_delayed_ = imuBuffer_.get_oldest(); // TODO: neccessary or can we use imu_sample_new directly?


        // check if filter is initialized
        if (!filter_initialised_) {
            filter_initialised_ = initializeFilter();
            if (!filter_initialised_) {
                return;
            }
        }


        // %%% KF states and matrices %%%

        // % INS states
        vec3 p_ins = state_.pos;
        vec3 v_ins = state_.vel;
        vec3 acc_bias_ins = state_.accel_bias;
        quat q_ins = state_.quat_nominal;
        vec3 gyro_bias_ins = state_.gyro_bias;


        // % WGS-84 gravity model
        g = gravity(mu);
        g_n << 0, 0, g;
        
        // Constants
        O3.setZero(3,3);
        O_13.setZero(1,3);
        O_19.setZero(1,9);
        I3.setIdentity(3,3);
        I15.setIdentity(15,15);

        // Reference vector
        v01 << 0, 0, 1; 

        // Rotation matrix 
        q_ins.normalize(); // TODO: Do we need to normalize here?
        R = q_ins.toRotationMatrix(); 

        // Bias compensated IMU measurements
        vec3 f_ins = imu_sample_delayed_.delta_vel - acc_bias_ins; 
        vec3 w_ins = imu_sample_delayed_.delta_ang - gyro_bias_ins;
        
        // * Discrete-time KF matrices *

        // TODO: move the static parts of the matrices to initialization?

        A.block(0,0,3,3) = O3;
        A.block(0,3,3,3) = I3;
        A.block(0,6,3,3) = O3;
        A.block(0,9,3,3) = O3;
        A.block(0,12,3,3) = O3;

        A.block(3,0,3,3) = O3;
        A.block(3,3,3,3) = O3;
        A.block(3,6,3,3) = -R;
        A.block(3,9,3,3) = -R*Smtrx(f_ins);
        A.block(3,12,3,3) = O3;

        A.block(6,0,3,3) = O3;
        A.block(6,3,3,3) = O3;
        A.block(6,6,3,3) = -(1/T_acc)*I3;
        A.block(6,9,3,3) = O3;
        A.block(6,12,3,3) = O3;

        A.block(9,0,3,3) = O3;
        A.block(9,3,3,3) = O3;
        A.block(9,6,3,3) = O3;
        A.block(9,9,3,3) = -Smtrx(w_ins);
        A.block(9,12,3,3) = O3;

        A.block(12,0,3,3) = O3;
        A.block(12,3,3,3) = O3;
        A.block(12,6,3,3) = O3;
        A.block(12,9,3,3) = O3;
        A.block(12,12,3,3) = -(1/T_gyro)*I3;


        // TODO: use measured delta time dt or fixed sampling time?
    
        Ad =  I15 + h*A + (1/2)*(h*A)*(h*A);   

        // linearization of heading measurement
        vec3 a = (2/q_ins.w()) * vec3(q_ins.x(),q_ins.y(),q_ins.z()); // 2 x Gibbs vector
        double u = 2 * ( a.x()*a.y() + 2*a.z() ) / ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) );
        double du = 1 / (1 + pow(u,2));
        vec3 c_psi = du * ( 1 / pow( (4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2)), 2) ) * 
                vec3( -2*( pow(a.x(),2) + pow(a.z(),2) - 4 )*a.y() + pow(a.y(),3) + 4*a.x()*a.z(),
                       2*( pow(a.y(),2) - pow(a.z(),2) + 4 )*a.x() + pow(a.x(),3) + 4*a.y()*a.z(),
                       4*( pow(a.z(),2) + a.x()*a.y()*a.z() + pow(a.x(),2) - pow(a.y(),2) + 4 ));


        // We asssume no velocity meausurements available 

        // NED positions
        Cd.block(0,0,3,3) = I3;
        Cd.block(0,3,3,3) = O3;
        Cd.block(0,6,3,3) = O3;
        Cd.block(0,9,3,3) = O3;
        Cd.block(0,12,3,3) = O3;

        // Gravity
        Cd.block(3,0,3,3) = O3;
        Cd.block(3,3,3,3) = O3;
        Cd.block(3,6,3,3) = O3;
        Cd.block(3,9,3,3) = Smtrx(R.transpose()*v01);
        Cd.block(3,12,3,3) = O3;

        // Camera heading
        Cd.block(6,0,3,3) = O_19;
        Cd.block(6,9,3,3) = c_psi.transpose();
        Cd.block(6,12,3,3) = O_13;

        
        Ed.block(0,0,3,3) = O3;
        Ed.block(0,3,3,3) = O3;
        Ed.block(0,6,3,3) = O3;
        Ed.block(0,9,3,3) = O3;

        Ed.block(3,0,3,3) = -R;
        Ed.block(3,3,3,3) = O3;
        Ed.block(3,6,3,3) = O3;
        Ed.block(3,9,3,3) = O3;

        Ed.block(6,0,3,3) = O3;
        Ed.block(6,3,3,3) = I3;
        Ed.block(6,6,3,3) = O3;
        Ed.block(6,9,3,3) = O3;

        Ed.block(9,0,3,3) = O3;
        Ed.block(9,3,3,3) = O3;
        Ed.block(9,6,3,3) = -I3;
        Ed.block(9,9,3,3) = O3;

        Ed.block(12,0,3,3) = O3;
        Ed.block(12,3,3,3) = O3;
        Ed.block(12,6,3,3) = O3;
        Ed.block(12,9,3,3) = I3;

        
        
        // %%% kalman filter algorithm %%%


        // check if fresh vision updates exists
        
        // * alternative 1 *
        
        /*
        if( (time_last_cam_pose_ > current_cam_pose_time) && (time_last_cam_pose_ > 0) ){
            cam_pose_ready_ = true;
            current_cam_pose_time = time_last_cam_pose_; 
        }
        else{
            cam_pose_ready_ = false;
        }
        */
    
        // * alternative 2 *

        // * if available:
        // -> pop first cam pose with timestamp older than imu timestamp
        // -> remove old data in buffer (set tail to the item that comes after the one we removed)
        // -> return true
        cam_pose_ready_ = camPoseBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &cam_pose_delayed_); 
        
        // no camera pose measurements available (no aiding)
        if(!cam_pose_ready_){

            P_hat = P_prd;

        }

        // camera pose measurements available (INS aiding)
        else{

            // KF gain
            K = P_prd * Cd.transpose() * (Cd * P_prd * Cd.transpose() + Rd).inverse(); 
            IKC = I15 - K * Cd;

            // extract latest camera pose measurements
            cameraPoseSample cam_pose_newest = camPoseBuffer_.get_newest(); 
            vec3 y_pos = cam_pose_newest.posNED;
            quat cam_quat = cam_pose_newest.quatNED; 
            vec3 cam_euler = q2euler(cam_quat); // extract euler angles
            double y_psi = cam_euler(2);
            

            // estimation error
            vec3 v1 = -f_ins/g; // gravity vector
            v1 = v1 / sqrt( v1.transpose() * v1 );

            vec3 eps_pos = y_pos - p_ins;
            vec3 eps_g   = v1 - R.transpose() * v01; 

            // smallest signed angle
            double eps_psi = ssa( y_psi - atan(u) ); // (in radians)
      
            // we assume no velocity measurements here
            eps.block(0,0,3,1) = eps_pos;
            eps.block(0,3,3,1) = eps_g;
            eps(0,6) = eps_psi;

            // corrector
            delta_x_hat = K * eps;
            P_hat = IKC * P_prd * IKC.transpose() + K * Rd * K.transpose();

            // error quaternion (2 x Gibbs vector)
            vec3 delta_a = delta_x_hat.block(10,0,3,1);
            vec4 delta_q_hat_vec = 1/(sqrt(4 + delta_a.transpose() * delta_a)) * vec4(2,delta_a.x(),delta_a.y(),delta_a.z());
            quat delta_q_hat = quat(delta_q_hat_vec(0),delta_q_hat_vec(1),delta_q_hat_vec(2),delta_q_hat_vec(3)); // vec4 to quat

            // INS reset
            p_ins = p_ins + delta_x_hat.block(0,0,3,1);	                 // position
	        v_ins = v_ins + delta_x_hat.block(0,3,3,1);			         // velocity
	        acc_bias_ins = acc_bias_ins + delta_x_hat.block(0,6,3,1);    // acc bias
	        gyro_bias_ins = gyro_bias_ins + delta_x_hat.block(0,13,3,1); // gyro bias
            
            q_ins = quatprod(q_ins, delta_q_hat);  // Schur product   
            q_ins.normalize();                     // normalization
               
        }

        // predictor
        P_prd = Ad * P_hat * Ad.transpose() + Ed * Qd * Ed.transpose();

        // INS propagation: x_ins[k+1]
        vec3 a_ins = R * f_ins + g_n;                                                          // linear acceleration
        p_ins = p_ins + h * v_ins + pow(h,2)/2 * a_ins;                                        // exact discretization
        v_ins = v_ins + h * a_ins;                                                             // exact discretization        
        vec4 q_ins_vec = Tquat(w_ins*h).exp() * vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()); // exact discretization
        q_ins = quat(q_ins_vec(0),q_ins_vec(1),q_ins_vec(2),q_ins_vec(3));                     // vec4 to quat 
        q_ins.normalize();                                                                     // normalization

  
        // update INS states
        state_.pos = p_ins;
        state_.vel = v_ins;
        state_.accel_bias = acc_bias_ins;
        state_.quat_nominal = q_ins;
        state_.gyro_bias = gyro_bias_ins;

        // reset vision update flag for each imu update 
        cam_pose_ready_ = false;
      
    }


    // gravity model (MSS toolbox)
    double MEKF::gravity(double mu){
        return 9.7803253359 * ( 1 + 0.001931850400 * pow(sin(mu),2) ) / sqrt( 1 - 0.006694384442 * pow(sin(mu),2) );
    }

    // Smallest signed angle (MSS toolbox)
    double MEKF::ssa(double angle){
        return ((angle + M_PI) - floor((angle + M_PI)/(2*M_PI)) * 2*M_PI) - M_PI;
    }



}


