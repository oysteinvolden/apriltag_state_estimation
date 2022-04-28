#include <mekf/mekf.h>

// libraries for file streams
#include <iostream>
#include <fstream>
#include <sstream>



namespace mekf{

    MEKF::MEKF(){
      
        // initialize INS - TODO: use specific initialization from SBG?        
        
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


       // allocate INS pos buffer
        sbgNavBuffer_.allocate(sbg_nav_buffer_length_);
        for (int i = 0; i < sbg_nav_buffer_length_; i++) {
            sbgNavSample sbg_nav_sample_init = {};
            sbgNavBuffer_.push(sbg_nav_sample_init);
        }

        // allocate INS quat buffer
        sbgQuatBuffer_.allocate(sbg_quat_buffer_length_);
        for (int i = 0; i < sbg_quat_buffer_length_; i++) {
            sbgQuatSample sbg_quat_sample_init = {};
            sbgQuatBuffer_.push(sbg_quat_sample_init);
        }
        
        // set to false, so we can initialize
        filter_initialised_ = false;

        // set to false until we have a valid MEKF estimate to publish
        publish_MEKF_ = false;

    }


    bool MEKF::initializeFilter(){

        // *** initializa covariance matrices ***

        // put the squares of the typical values of the the state components + margin

        
        P_prd.diagonal() << 0.000016, 0.000016, 0.000016, // delta pos in NED [m] 
                            0.000016, 0.000016, 0.000016, // delta velocity in NED [m/s]
                            0.000003,   0.000003,   0.000003, // delta acceleration bias [m/s^2]  
                            0.00000005,  0.00000005,  0.00000005, // delta a 
                            0.00000025, 0.00000025, 0.00000025; // delta gyro bias [rad]


        // ADIS imu data
    
        double sigma_w_acc = 0.0001333; // velocity random walk
        double sigma_b_acc = 8.3212184588*10e-7; // velocity in run instability bias
        double sigma_w_ars = 0.000026179938; // angular random walk
        double sigma_b_ars = 6.54728501*10e-8; // angular in run instability bias 
        

        // initialize process weights - v, acc_bias, w, gyro_bias (v, acc_bias, w, ars_bias)        
        Qd.diagonal() << sigma_w_acc, sigma_w_acc, sigma_w_acc,
         sigma_b_acc, sigma_b_acc, sigma_b_acc,
         sigma_w_ars, sigma_w_ars, sigma_w_ars,
         sigma_b_ars, sigma_b_ars, sigma_b_ars;
        
        Qd = 3 * h * Qd; // discretize + margin

 
        // this one is good
        Rd.diagonal() << 0.001, 0.001, 0.001, // p
                         0.001, 0.001, 0.001, // acc
                         0.0001;       // psi                 
        
        Rd = 0.001*Rd; // adis imu
        

        // %%%%% initialize state %%%%

        // extract SBG nav sample
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        y_pos = sbg_nav_newest.posNED;
	    y_vel = sbg_nav_newest.vel;
    
        // check is SBG sample has arrived - TODO: do another check?
        if(y_pos.x() == 0 && y_pos.y() == 0 && y_pos.z() == 0){
            return false;
        }
        else
        {
            // extract sbg quat
            sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
            y_quat = sbg_quat_newest.quatNED;

            //y_quat.normalize();

            state_.quat_nominal = y_quat;
            state_.vel = y_vel; 
            state_.pos = y_pos;
            state_.gyro_bias = vec3(0,0,0); 
            state_.accel_bias = vec3(0,0,0);

            return true;
        }
	        

    }

    
    void MEKF::updateCamPose(const vec3& cam_pos, const quat& cam_quat, uint64_t time_usec){

        // copy required data
        cam_pose_sample_new_.posNED = cam_pos;
        cam_pose_sample_new_.quatNED = cam_quat;
        cam_pose_sample_new_.posErr = 0.05; // TODO: neccessary?
        cam_pose_sample_new_.angErr = 0.05; // TODO: neccessary?
        cam_pose_sample_new_.time_us = time_usec; // TODO: do we need to subtract camera delay here?

        // update last time
        time_last_cam_pose_ = time_usec;

        // push pose to buffer
        camPoseBuffer_.push(cam_pose_sample_new_);

    }
    


    void MEKF::updateSbgNav(const vec3& sbg_pos, const vec3& sbg_vel, uint64_t time_usec){

        // copy required data
        sbg_nav_sample_new_.posNED = sbg_pos;
	    sbg_nav_sample_new_.vel = sbg_vel;
        sbg_nav_sample_new_.time_us = time_usec; // TODO: do we need to subtract delay here?

        // update last time
        time_last_sbg_pos_ = time_usec;

        // push pose to buffer
        sbgNavBuffer_.push(sbg_nav_sample_new_); 

    }

    void MEKF::updateSbgQuat(const quat& sbg_quat, uint64_t time_usec){

        // copy required data
        sbg_quat_sample_new_.quatNED = sbg_quat;
        sbg_quat_sample_new_.time_us = time_usec; // TODO: do we need to subtract delay here?

        // update last time
        time_last_sbg_quat_ = time_usec;

        // push pose to buffer
        sbgQuatBuffer_.push(sbg_quat_sample_new_); 

    }


    // check if estimated state is accurate enough to be used in feedback control (i.e., be pulished and used)
    void MEKF::publish_MEKF(uint64_t time_us){

        
        // get latest SBG INS sample
        sbgNavSample sbg_nav_sample = sbgNavBuffer_.get_newest();
        sbgQuatSample sbg_quat_sample = sbgQuatBuffer_.get_newest();
        vec3 sbg_pos = sbg_nav_sample.posNED;
        quat sbg_quat = sbg_quat_sample.quatNED;

        
        // get latest MEKF sample
        quat mekf_quat = getQuat();
        vec3 mekf_pos = getPosition();
        //const vec3 velocity = mekf_.getVelocity(); // TODO: assume velocity is ok?

    
        // quat to euler
        geometry_msgs::Quaternion mekf_quat_msg, sbg_quat_msg;

        sbg_quat_msg.x = sbg_quat.x();
	    sbg_quat_msg.y = sbg_quat.y();
	    sbg_quat_msg.z = sbg_quat.z();
	    sbg_quat_msg.w = sbg_quat.w();

        mekf_quat_msg.x = mekf_quat.x();
	    mekf_quat_msg.y = mekf_quat.y();
	    mekf_quat_msg.z = mekf_quat.z();
	    mekf_quat_msg.w = mekf_quat.w();

        
	    tf::Quaternion tf_quat_sbg, tf_quat_mekf;
	    tf::quaternionMsgToTF(mekf_quat_msg, tf_quat_mekf);
        tf::quaternionMsgToTF(sbg_quat_msg, tf_quat_sbg);


        double roll_sbg, pitch_sbg, yaw_sbg, roll_mekf, pitch_mekf, yaw_mekf;
        tf::Matrix3x3(tf_quat_sbg).getRPY(roll_sbg, pitch_sbg, yaw_sbg); 
        tf::Matrix3x3(tf_quat_mekf).getRPY(roll_mekf, pitch_mekf, yaw_mekf); 



        // 1. Check if yaw/pos threshold is satisfied (we use SBG INS as ground truth)
        if((abs(sbg_pos.x() - mekf_pos.x()) <  x_pos_pub_thresh_) && (abs(sbg_pos.y() - mekf_pos.y()) <  y_pos_pub_thresh_) && (abs(yaw_sbg - yaw_mekf) < yaw_pub_thresh_)){

            // 2. Check for consistently estimates over time

            // increment counter
            accepted_mekf_estimates_++;

            // If more than X consecutive mekf estimates satisfies (2), we publish mekf instead of SBG INS 
            // we also require that the filter use camera as source and not SBG
            if((accepted_mekf_estimates_ >= consecutive_mekf_thresh_) && (!use_sbg_ins) && (!publish_MEKF_)){

                publish_MEKF_ = true;  
                ROS_INFO("********************************");
                ROS_INFO("We publish MEKF now");
                ROS_INFO("********************************");
		        std::cout << "time stamp MEKF published: " << time_us << std::endl;

            }
                
        }
        else
        {
            // reset counter (since we require X consecutive estimates satisfying (2))
            accepted_mekf_estimates_ = 0;
        }
    

    }
    
    
    


    // Navigation management 
    // * Decide if we use SBG INS or camera pose based on
    // * 1) Euclidean distance from reference marker (i.e., origo in NED)
    // * 2) Camera pose accuracy
    // * 3) Consistently camera measurements over time
 
    void MEKF::runNavigationManagement(uint64_t time_us){


        // get latest SBG INS sample
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
        vec3 sbg_pos = sbg_nav_newest.posNED;
        quat sbg_quat = sbg_quat_newest.quatNED;

        // get latest camera pose sample
        cameraPoseSample cam_pose_newest = camPoseBuffer_.get_newest(); 
        vec3 cam_pos = cam_pose_newest.posNED;
        quat cam_quat = cam_pose_newest.quatNED;

        // quaternion to euler angles        
        geometry_msgs::Quaternion cam_quat_msg, sbg_quat_msg;

        sbg_quat_msg.x = sbg_quat.x();
	    sbg_quat_msg.y = sbg_quat.y();
	    sbg_quat_msg.z = sbg_quat.z();
	    sbg_quat_msg.w = sbg_quat.w();

        cam_quat_msg.x = cam_quat.x();
	    cam_quat_msg.y = cam_quat.y();
	    cam_quat_msg.z = cam_quat.z();
	    cam_quat_msg.w = cam_quat.w();

        
	    tf::Quaternion tf_quat_sbg, tf_quat_cam;
	    tf::quaternionMsgToTF(cam_quat_msg, tf_quat_cam);
        tf::quaternionMsgToTF(sbg_quat_msg, tf_quat_sbg);


        double roll_sbg, pitch_sbg, yaw_sbg, roll_cam, pitch_cam, yaw_cam;
        tf::Matrix3x3(tf_quat_sbg).getRPY(roll_sbg, pitch_sbg, yaw_sbg); 
        tf::Matrix3x3(tf_quat_cam).getRPY(roll_cam, pitch_cam, yaw_cam); 
        
  
        // 1. Are we inside the 2D euclidean distance region and do we use the SBG INS?
        if((sqrt(pow(sbg_pos.x(),2) + pow(sbg_pos.y(),2)) < euclidean_thresh_) && use_sbg_ins == true){
            
            // 2. Check if yaw/pos threshold is satisfied (we use SBG INS as ground truth)
            if( (abs(sbg_pos.x() - cam_pos.x()) <  x_pos_thresh_) && (abs(sbg_pos.y() - cam_pos.y()) <  y_pos_thresh_) && (abs(yaw_sbg - yaw_cam) < yaw_thresh_) ){
               
                // 3. Check for consistently measurements over time

                // increment counter
                accepted_cam_measurements_++;

                // if more than X consecutive camera pose measurements satisfies (2), we use camera pose instead of SBG INS 
                if(accepted_cam_measurements_ >= consecutive_cam_meas_thresh_){
                   
	   	            use_sbg_ins = false; // once set to false, we are finished and feed camera pose into the INS 
		            //publish_MEKF_ = true;
                    ROS_INFO("***********************************");
		            ROS_INFO("We use Camera Pose measurements now");
		            ROS_INFO("***********************************");
		            std::cout << "time stamp cam pose in mekf: " << time_us << std::endl;
                }
                
            }
            else
            {
                // reset counter (since we require X consecutive camera pose measurements satisfying (2))
                accepted_cam_measurements_ = 0;
            }
            
        }


    }
    



    void MEKF::run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, double dt){

	    //std::cout << "TEST" << std::endl;

        // TEST - check if cam/sbg quat is normalized
        cameraPoseSample cam_pose_test = camPoseBuffer_.get_newest();
        quat cam_quat_new = cam_pose_test.quatNED;

        sbgQuatSample sbg_quat_test = sbgQuatBuffer_.get_newest();
        quat sbg_quat_new = sbg_quat_test.quatNED;

        double quat_norm_cam = sqrt ( pow(cam_quat_new.w(),2) + pow(cam_quat_new.x(),2) + pow(cam_quat_new.y(),2) + pow(cam_quat_new.z(),2) ) ;
        double quat_norm_sbg = sqrt ( pow(sbg_quat_new.w(),2) + pow(sbg_quat_new.x(),2) + pow(sbg_quat_new.y(),2) + pow(sbg_quat_new.z(),2) ) ;


        // check if quaternion from camera measurements is satisfied
        if(!(quat_norm_cam >= 0.99 && quat_norm_cam <= 1.01)){
		    //std::cout << "CAM QUAT UNIT TEST" << std::endl;    
		    return;
        }
	
        /*	
        // check if quaternion from sbg measurements is satisfied
        if(!(quat_norm_sbg >= 0.99 && quat_norm_sbg <= 1.01)){
            	std::cout << "sbg quat test" << std::endl;
		return;
        }
	    */
	

        // copy imu data
        imu_sample_new_.delta_ang = vec3(ang_vel.x(), ang_vel.y(), ang_vel.z()); // current yaw rate [rad/s] // TODO: change name
        imu_sample_new_.delta_vel = vec3(lin_acc.x(), lin_acc.y(), lin_acc.z()); // current linear acceleration [m/s^2]

        imu_sample_new_.delta_ang_dt = dt;
        imu_sample_new_.delta_vel_dt = dt;
        imu_sample_new_.time_us = time_us;
        time_last_imu_ = time_us; 

        // push to buffer
        imuBuffer_.push(imu_sample_new_); 

        // get the oldest data from the buffer
        imu_sample_delayed_ = imuBuffer_.get_oldest(); // TODO: neccessary or can we use imu_sample_new directly?
        //imu_sample_delayed_ = imuBuffer_.get_newest();



        // *** TEST validity of IMU measurements ***

	    if(abs(imu_sample_delayed_.delta_ang.x()) < 0.0001){
            //std::cout << "TEST 1" << std::endl;
	        return;
        }

	    if(abs(imu_sample_delayed_.delta_vel.x()) < 0.0001){
		    //std::cout << "TEST 2" << std::endl;
 	        return;
        }

	
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
        A.block(9,12,3,3) = -I3; 

        A.block(12,0,3,3) = O3;
        A.block(12,3,3,3) = O3;
        A.block(12,6,3,3) = O3;
        A.block(12,9,3,3) = O3; 
        A.block(12,12,3,3) = -(1/T_gyro)*I3;

   

        Ad =  I15 + h*A + 0.5*(h*A)*(h*A); 
 
      
        // linearization of heading measurement
        vec3 a = (2/q_ins.w()) * vec3(q_ins.x(),q_ins.y(),q_ins.z()); // 2 x Gibbs vector

        double u = 2 * ( a.x()*a.y() + 2*a.z() ) / ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) );

        double du = 1 / (1 + pow(u,2));
        vec3 c_psi = du * ( 1 / pow( ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) ), 2) ) * 
                vec3( -2*(( pow(a.x(),2) + pow(a.z(),2) - 4 )*a.y() + pow(a.y(),3) + 4*a.x()*a.z()),
                       2*(( pow(a.y(),2) - pow(a.z(),2) + 4 )*a.x() + pow(a.x(),3) + 4*a.y()*a.z()),
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
        Cd.block(6,0,1,9) = O_19; 
        Cd.block(6,9,1,3) = c_psi.transpose();
        Cd.block(6,12,1,3) = O_13;
        
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

        Ed = h * Ed;




  
        // %%% kalman filter algorithm %%%


        // check if fresh vision updates exists
        
        // * if available:
        // -> pop first cam pose with timestamp older than imu timestamp
        // -> remove old data in buffer (set tail to the item that comes after the one we removed)
        // -> return true
        
        cam_pose_ready_ = camPoseBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &cam_pose_delayed_); 

	    // TODO: for pure camera testing (no sbg available), comment out runNavigationManagment() and set use_sbg_ins to false
      
        // We only run navigation management when a camera pose is available 
        if(cam_pose_ready_){
            runNavigationManagement(time_us); 
        }
        
        // no camera pose measurements available (no aiding)     
        if(!cam_pose_ready_){

            P_hat = P_prd;

        }
        
        // camera pose measurements available (INS aiding)
        else{
            K.setZero(15,7); // TODO: initialize before?

            // KF gain
            K = P_prd * Cd.transpose() * (Cd * P_prd * Cd.transpose() + Rd).inverse(); 


            IKC.setZero(15,15); // TODO: initialize before?
            IKC = I15 - K * Cd;

    
            // if camera pose measurements is available but not accurate enough yet
            if(use_sbg_ins){

                // extract latest SBG ins measurements
                sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
                sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
                y_pos = sbg_nav_newest.posNED;
                y_quat = sbg_quat_newest.quatNED;


                // quat to euler         
                geometry_msgs::Quaternion quat_msg;
                quat_msg.x = y_quat.x();
                quat_msg.y = y_quat.y();
                quat_msg.z = y_quat.z();
                quat_msg.w = y_quat.w();

                // quat -> tf
                tf::Quaternion quatTf;
                tf::quaternionMsgToTF(quat_msg, quatTf);

                double roll, pitch, yaw;
                tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw); 
                      
                y_psi = yaw;
            }

            // if camera pose measurements is accurate enough
            else{

                
                // extract latest camera pose measurements
                cameraPoseSample cam_pose_newest = camPoseBuffer_.get_newest(); 
                y_pos = cam_pose_newest.posNED;
                y_quat = cam_pose_newest.quatNED; 

                // quat to euler 
                geometry_msgs::Quaternion quat_msg;
                quat_msg.x = y_quat.x();
                quat_msg.y = y_quat.y();
                quat_msg.z = y_quat.z();
                quat_msg.w = y_quat.w();

                // quat -> tf
                tf::Quaternion quatTf;
                tf::quaternionMsgToTF(quat_msg, quatTf);

                double roll, pitch, yaw;
                tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw); 
                 
                y_psi = yaw;
            }

                        
            // ************************************


            // estimation error
            vec3 v1 = -f_ins/g; // gravity vector
            v1 = v1 / sqrt( v1.transpose() * v1 );

            vec3 eps_pos = y_pos - p_ins;
            vec3 eps_g = v1 - R.transpose() * v01; 

            // smallest signed angle
            double eps_psi = ssa(y_psi - atan(u)); // (in radians)


            // we assume no velocity measurements here
            eps.block(0,0,3,1) = eps_pos;
            eps.block(3,0,3,1) = eps_g;
            eps(6,0) = eps_psi;

            // corrector
            delta_x_hat = K * eps;
            P_hat = IKC * P_prd * IKC.transpose() + K * Rd * K.transpose();


            // error quaternion (2 x Gibbs vector)
            vec3 delta_a = delta_x_hat.block(9,0,3,1);
            vec4 delta_q_hat_vec = 1/(sqrt(4 + delta_a.transpose() * delta_a)) * vec4(2,delta_a.x(),delta_a.y(),delta_a.z());
            quat delta_q_hat = quat(delta_q_hat_vec(0),delta_q_hat_vec(1),delta_q_hat_vec(2),delta_q_hat_vec(3)); // vec4 to quat

            // INS reset
            p_ins = p_ins + delta_x_hat.block(0,0,3,1);	                 // position
	        v_ins = v_ins + delta_x_hat.block(3,0,3,1);			         // velocity
	        acc_bias_ins = acc_bias_ins + delta_x_hat.block(6,0,3,1);    // acc bias
	        gyro_bias_ins = gyro_bias_ins + delta_x_hat.block(12,0,3,1); // gyro bias         
            q_ins = quatprod(q_ins, delta_q_hat);                        // schur product   
            q_ins.normalize();                                           // normalization

        }


        // predictor
        P_prd = Ad * P_hat * Ad.transpose() + Ed * Qd * Ed.transpose();


        // INS propagation: x_ins[k+1]
        vec3 a_ins = R * f_ins + g_n;                                                          // linear acceleration
        p_ins = p_ins + h * v_ins + pow(h,2)/2 * a_ins;                                        // exact discretization
        v_ins = v_ins + h * a_ins;                                                             // exact discretization   
        vec4 q_ins_vec = (h*Tquat_vec3(w_ins)).exp() * vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()); // exact discretization
        //vec4 q_ins_vec = vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()) + h*Tquat_quat(q_ins) * w_ins; // Euler's method (alternative)

        q_ins = quat(q_ins_vec(0),q_ins_vec(1),q_ins_vec(2),q_ins_vec(3));                     // vec4 to quat 
        q_ins.normalize();                                                                     // normalization

        // update INS states
        state_.pos = p_ins;
        state_.vel = v_ins;
        state_.accel_bias = acc_bias_ins;
        state_.quat_nominal = q_ins;
        state_.gyro_bias = gyro_bias_ins;


        // quaternion to euler angles       
        geometry_msgs::Quaternion quat_msg2;
        quat_msg2.x = q_ins.x();
	    quat_msg2.y = q_ins.y();
	    quat_msg2.z = q_ins.z();
	    quat_msg2.w = q_ins.w();

	    // quat -> tf
	    tf::Quaternion quat3;
	    tf::quaternionMsgToTF(quat_msg2, quat3);

        double roll_est, pitch_est, yaw_est;
        tf::Matrix3x3(quat3).getRPY(roll_est, pitch_est, yaw_est); 
    
        // reset vision flag after each INS update 
        cam_pose_ready_ = false; // TODO: same procedure for SBG pose ready if used 


        // assign publish_MEKF_ to true if mekf estimates are accurate enough (so we can publish mekf instead of sbg)
        publish_MEKF(time_us); 

      
    }




    // gravity model (MSS toolbox)
    double MEKF::gravity(double mu){
        return 9.7803253359 * ( 1 + 0.001931850400 * pow(sin(mu),2) ) / sqrt( 1 - 0.006694384442 * pow(sin(mu),2) );
    }

    // Smallest signed angle (MSS toolbox)
    double MEKF::ssa(double angle){
        return ((angle + M_PI) - floor((angle + M_PI)/(2*M_PI)) * 2*M_PI) - M_PI;
    }

    


    quat MEKF::getQuat(){
        return state_.quat_nominal;
    }

    vec3 MEKF::getPosition(){
        return state_.pos;
    }

    vec3 MEKF::getVelocity(){
        return state_.vel;
    }

    uint64_t MEKF::getImuTime(){
        return time_last_imu_; 
    }




}


