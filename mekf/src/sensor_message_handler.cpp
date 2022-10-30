#include <mekf/sensor_message_handler.h>

// libraries for file streams
#include <iostream>
#include <fstream>
#include <sstream>



namespace mekf{

    MessageHandler::MessageHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
        


        ROS_INFO("Subscribing to IMU");
        
	    subImu_ = nh_.subscribe("/imc/adis_imu", 1, &MessageHandler::imuCallback, this); // ADIS IMU (DUNE) 

        ROS_INFO("Subscribing to camera pose");
        
        subCameraPose_ = nh_.subscribe("/apriltag_bundle_pose", 1, &MessageHandler::cameraPoseCallback, this);

        ROS_INFO("Subscribing to SBG EKF NAV and SBG EKF QUAT");

        // *** SBG ***

        // SBG EKF NAV
        subEkfNav_ = nh_.subscribe("/sbg/ekf_nav", 1, &MessageHandler::ekfNavCallback, this);

        // SBG EKF Euler
        subEkfEuler_ = nh_.subscribe("/sbg/ekf_euler", 1, &MessageHandler::ekfEulerCallback, this);
        
        // *** Publishers ***
        
        pubEstimatedNav_ = nh_.advertise<sbg_driver::SbgEkfNav>("/sbg/mekf_nav", 1);
        pubEstimatedEuler_ = nh_.advertise<sbg_driver::SbgEkfEuler>("/sbg/mekf_euler", 1);

        int publish_rate = publish_rate_;
        pubTimerNav_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &MessageHandler::publishEstimatedNav, this);
        pubTimerEuler_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &MessageHandler::publishEstimatedEuler, this);
        

    }


    // -------------------------------------------------
    // %%% IMU %%%%
    // -------------------------------------------------


    // Rotate IMU to align with body frame
    Eigen::Transform<double,3,Eigen::Affine> MessageHandler::getImuToBodyT(){

        // %%%% from IMU to body %%%%

        // roll, pitch, yaw - order: about X Y Z respectively 
        double roll=0, pitch=0, yaw=M_PI;
        Eigen::Quaternion<double> R_imu_to_body;
        R_imu_to_body = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());
        Eigen::Transform<double,3,Eigen::Affine> T_imu_to_body(R_imu_to_body);

        return T_imu_to_body; 
    }
    

    
    // We only transform linear acceleration and angular velocity
    sensor_msgs::Imu MessageHandler::imuTransform(const sensor_msgs::ImuConstPtr &imu_in, const Eigen::Transform<double,3,Eigen::Affine> &T){

        // copy header
        sensor_msgs::Imu imu_out;
        imu_out.header = imu_in->header;

        // angular velocity
        Eigen::Vector3d vel = T * Eigen::Vector3d(imu_in->angular_velocity.x, imu_in->angular_velocity.y, imu_in->angular_velocity.z);

        imu_out.angular_velocity.x = vel.x();
        imu_out.angular_velocity.y = vel.y();
        imu_out.angular_velocity.z = vel.z();

        // linear acceleration
        Eigen::Vector3d acc = T * Eigen::Vector3d(imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z);

        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        return imu_out; 
    }


    
    // *** IMU Callback ***
    // -----------------------------
    // * Take in imu messages 
    // * Reduce frequency from X Hz to Y Hz 
    // * Transform imu data to body frame
    // * run kalman filter with imu data
    // ------------------------------


    // ADIS IMU
    
    void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

            if(prevStampImu_.sec > 0){

                    
                if(!init_){
                        init_ = true;
                        ROS_INFO("Initialized MEKF");
                }   
                
                // sample time
		        mekf_.h = (imuMsg->header.stamp - prevStampImu_).toSec(); // We use sampling time (h) with small variations 
       
                // imu to body transformation
                Eigen::Transform<double,3,Eigen::Affine> imuToBodyT = getImuToBodyT();
    
                // transform imu data to body frame
                sensor_msgs::Imu imuInBody = imuTransform(imuMsg, imuToBodyT);

                // get measurements
                vec3 ang_vel = vec3(imuInBody.angular_velocity.x, imuInBody.angular_velocity.y, imuInBody.angular_velocity.z);
                vec3 lin_acc = vec3(imuInBody.linear_acceleration.x, imuInBody.linear_acceleration.y, imuInBody.linear_acceleration.z);
                
                // run kalman filter
		        mekf_.run_mekf(ang_vel, lin_acc, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), mekf_.h);
            
            }

            prevStampImu_ = imuMsg->header.stamp;

    }
    
    


    



    // -------------------------------------------------
    // %%% Camera %%%%
    // -------------------------------------------------


    geometry_msgs::PoseWithCovarianceStamped MessageHandler::cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn){


        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header = cameraPoseIn->header;


        // %%% Part 1: BODY transformations 

        // We use static transform from camera to center of vehicle in camera frame instead %%% 
        // Measured offsets between camera and center of vehicle (in camera frame):
        // x = 0.06 m, y = 0.3115 m, z = 0.033 m 
   
        // left camera (default)
        pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - 0.06; 
        pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.3115; 
        pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z - 0.033; 
        

        // no rotation yet
        pose.pose.pose.orientation.x = cameraPoseIn->pose.pose.orientation.x;
        pose.pose.pose.orientation.y = cameraPoseIn->pose.pose.orientation.y;
        pose.pose.pose.orientation.z = cameraPoseIn->pose.pose.orientation.z;
        pose.pose.pose.orientation.w = cameraPoseIn->pose.pose.orientation.w;


        // %%% Part 2:  inverse -> pose is expressed relative to tag frame instead of body IMU/camera frame %%%

        // pose -> tf
        tf::Stamped<tf::Transform> tag_transform;
        tf::poseMsgToTF(pose.pose.pose, tag_transform);

        // tf -> tf inverse
        tf::Transform tag_transform_inverse;
        tag_transform_inverse = tag_transform.inverse();

        // tf inverse -> pose inverse
        geometry_msgs::PoseWithCovarianceStamped pose_inverse;
        pose_inverse.header = cameraPoseIn->header;
        tf::poseTFToMsg(tag_transform_inverse, pose_inverse.pose.pose);

 

        // %%% step 3: rotate from tag frame to NED frame %%%

        tf2::Quaternion q_orig_1, q_rot_1, q_new_1;

        // extract original orientation
        tf2::convert(pose_inverse.pose.pose.orientation , q_orig_1);

        // set new rotation
        double r1=M_PI/2, p1=0, y1=M_PI/2; // roll, pitch, yaw - order: about X Y Z respectively
        q_rot_1.setRPY(r1,p1,y1);
        
        // rotate the previous orientation by q_rot and normalize
        q_new_1 = q_rot_1*q_orig_1;
        q_new_1.normalize();

        // pose NED
        geometry_msgs::PoseWithCovarianceStamped pose_NED;
        pose_NED.header = cameraPoseIn->header;

        // Stuff the new rotation back into the pose 
        tf2::convert(q_new_1, pose_NED.pose.pose.orientation);
        
        // update the position in the new reference frame
        pose_NED.pose.pose.position.x = pose_inverse.pose.pose.position.z;
        pose_NED.pose.pose.position.y = -pose_inverse.pose.pose.position.x;
        pose_NED.pose.pose.position.z = -pose_inverse.pose.pose.position.y;
  

        // %%% step 4: Align heading/position with NED frame %%%

        // * The yaw offset between the axis pointing out of the aprilTag and true north consist of two components: 
        // * 1. A fixed, measured rotation about Z axis (yaw): 227 degrees (if placed at fixed place at Brattørkaia)
        // * 2. A small rotation about Z axis (yaw) depending on which marker configuration is used (between 0 and 4 degrees)
        // * NB! The small rotation in (2) above is found by subtracting for the constant heading offset between ground truth SBG Ellipse INS and Apriltag (using offline data)
        // * The same small rotation is tested for multiple scenarios to show reproducibility -> E.g., 227 deg + 1.5 deg is closer to the true yaw offset 

        // pose rotated NED (to align apriltag with true north) 
        geometry_msgs::PoseWithCovarianceStamped pose_NED_rot;
        pose_NED_rot.header = cameraPoseIn->header; // copy header from original incoming message

        float yaw_offset, yaw_offset_1, yaw_offset_2;
    
        yaw_offset_1 = 227*(M_PI/180);
        yaw_offset_2 = 3.5*(M_PI/180); // We tune this parameter in real experiments  
        yaw_offset = yaw_offset_1 + yaw_offset_2;


        // %%% 4.1: find heading relative to true North %%%

        // * NB! Given that we have the tag frame as specified by AprilTag and rotate it by q_rot_1 to get the pose_NED orientation,
        // * we only need to rotate about z axis to find heading relative to true north.
        // * Hence, we do the following to find the NED heading angle:
        // * Subtract the measured Apriltag yaw and 90 degrees from the yaw offset (227 + e.g., 1.5 deg)
        // * Example: Heading = yaw_offset - 90 deg - apriltag_yaw_offset = (227 + 1.5) - 90 - apriltag_yaw_offset

        // * NB! Since we want to subtract the yaw measured by aprilTags, we go in opposite (negative since Z down) yaw direction

        // extract orientation
        tf2::Quaternion q_orig_2;
        tf2::convert(pose_NED.pose.pose.orientation , q_orig_2);

        // convert quat to rpy
        double roll, pitch, yaw, yaw_NED;
        tf2::Matrix3x3(q_orig_2).getRPY(roll, pitch, yaw);
        
        // compute yaw angle relative to north
        yaw_NED = yaw_offset - 90*(M_PI/180) - yaw;

        // we do not touch roll and pitch
        q_orig_2.setRPY(roll, pitch, yaw_NED);

        // TODO: do we need to normalize here?

        // Stuff the final rotation back into the pose 
        tf2::convert(q_orig_2, pose_NED_rot.pose.pose.orientation);


        // %%% 4.2: 2D rotation about yaw offset (psi_offset) to align position with NED %%%

        // NB! Z DOWN, hence we have to go in opposite direction (negative yaw_offset) so tag is aligned with true north
        float apriltag_x_ned, apriltag_y_ned;
        apriltag_y_ned = pose_NED.pose.pose.position.y*cos(-yaw_offset) - pose_NED.pose.pose.position.x*sin(-yaw_offset); 
        apriltag_x_ned = pose_NED.pose.pose.position.y*sin(-yaw_offset) + pose_NED.pose.pose.position.x*cos(-yaw_offset);

        // update the position in the final NED frame
        pose_NED_rot.pose.pose.position.x = apriltag_y_ned;
        pose_NED_rot.pose.pose.position.y = apriltag_x_ned;
        pose_NED_rot.pose.pose.position.z = pose_NED.pose.pose.position.z;
        
        return pose_NED_rot;
     
    }



    // *** Camera Pose Callback ***
    // -----------------------------
    // * Take in camera pose messages
    // * Transform them to NED
    // * update with pose sample
    // ------------------------------
    
    void MessageHandler::cameraPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseMsg){

        if(prevStampCameraPose_.sec > 0){

            // transform camera pose to NED
            geometry_msgs::PoseWithCovarianceStamped camPoseNED = cameraTransform(cameraPoseMsg);
   
            // get measurements - swap pos (x/y) - so compatible with GNSS
            vec3 cam_pos = vec3(camPoseNED.pose.pose.position.y,camPoseNED.pose.pose.position.x, camPoseNED.pose.pose.position.z);
            quat cam_quat = quat(camPoseNED.pose.pose.orientation.w, camPoseNED.pose.pose.orientation.x, camPoseNED.pose.pose.orientation.y, camPoseNED.pose.pose.orientation.z);

            cam_quat.normalize(); // TODO: neccessary?

	        //std::cout << cam_pos << std::endl;
	 
            // update pose sample
            //mekf_.updateCamPose(cam_pos, cam_quat, static_cast<uint64_t>(cameraPoseMsg->header.stamp.toSec()*1e6f)); // with time stamp from header
	        mekf_.updateCamPose(cam_pos, cam_quat, static_cast<uint64_t>(ros::Time::now().toSec()*1e6f)); // with ROS system time	    

        }

        prevStampCameraPose_ = cameraPoseMsg->header.stamp;
    }



    void MessageHandler::ekfNavCallback(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg){

        if(prevStampSbgEkfNav_.sec > 0){


            // convert from WGS84 to NED
            vec3 sbgPosNED = llh2flat(navSbgMsg); 
            vec3 sbg_pos = vec3(sbgPosNED.x(), sbgPosNED.y(), sbgPosNED.z()); 
	    
	        // get directly
	        vec3 sbg_vel = vec3(navSbgMsg->velocity.x,navSbgMsg->velocity.y, navSbgMsg->velocity.z); 
           
            // update sample
            mekf_.updateSbgNav(sbg_pos, sbg_vel, static_cast<uint64_t>(navSbgMsg->header.stamp.toSec()*1e6f));

            // publish output message directly if MEKF estimates are not used (25 hz)
            // because of the flag, we will either publish SBG OR MEKF estimates    
            if(!mekf_.publish_MEKF_){

                sbg_driver::SbgEkfNav navSbgMsgOut;

                // change header and sequence number and publish msg
                navSbgMsgOut.header.frame_id = "/sbg/mekf_nav";
                navSbgMsgOut.header.seq = trace_id_nav_++;
                navSbgMsgOut.header.stamp = ros::Time::now();


                navSbgMsgOut.position.x = (navSbgMsg->position.x)*(M_PI/180); // [rad] (since DUNE use rad)
                navSbgMsgOut.position.y = (navSbgMsg->position.y)*(M_PI/180); // [rad] (since DUNE use rad)
                navSbgMsgOut.position.z = (navSbgMsg->position.z + navSbgMsg->undulation); // [m] Height above WGS84 Ellipsoid = altitude + undulation

                // velocity is fine (units/format), so we just copy
                navSbgMsgOut.velocity.x = navSbgMsg->velocity.x; 
                navSbgMsgOut.velocity.y = navSbgMsg->velocity.y; 
                navSbgMsgOut.velocity.z = navSbgMsg->velocity.z; 

                // publish
                pubEstimatedNav_.publish(navSbgMsgOut);
            }
            
        }

        prevStampSbgEkfNav_ = navSbgMsg->header.stamp;

    }

    void MessageHandler::ekfEulerCallback(const sbg_driver::SbgEkfEulerConstPtr& eulerSbgMsg){

        if(prevStampSbgEkfEuler_.sec > 0){

            // convert from euler to quat
            vec3 sbg_euler = vec3(eulerSbgMsg->angle.x, eulerSbgMsg->angle.y, eulerSbgMsg->angle.z);
            quat sbg_quat = euler2q(sbg_euler);

            // update pose sample
            mekf_.updateSbgQuat(sbg_quat, static_cast<uint64_t>(eulerSbgMsg->header.stamp.toSec()*1e6f));
        
            // publish output message directly if MEKF estimates are not used (25 hz)
            // because of the flag, we will either publish SBG OR MEKF estimates    
            if(!mekf_.publish_MEKF_){

                sbg_driver::SbgEkfEuler eulerSbgMsgOut;

                // change header and sequence number and publish msg 
                eulerSbgMsgOut.header.frame_id = "/sbg/mekf_euler";
                eulerSbgMsgOut.header.seq = trace_id_euler_++;
                eulerSbgMsgOut.header.stamp = ros::Time::now();

                // euler angles are fine (units/format), so we just copy
                eulerSbgMsgOut.angle = eulerSbgMsg->angle;


                pubEstimatedEuler_.publish(eulerSbgMsgOut);

            }
            

        
        }

        prevStampSbgEkfEuler_ = eulerSbgMsg->header.stamp;

    }



    
    // convert from WGS-84 to NED (MSS toolbox)
    vec3 MessageHandler::llh2flat(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg){

        // reference position parameters

        // *****************************************************

        // * experiment 3 - brattørkaia location

	    // trinn 1
        double lat0 = 1.10723500;
        double lon0 = 0.18151317;
        double h0 = 41.404;
        
         // trinn 2
        /*
        double lat0 = 1.10723515;
        double lon0 = 0.18151361;
        double h0 = 41.863; 
        */

        // trinn 3
        /*
        double lat0 = 1.10723531;
        double lon0 = 0.18151397;
        double h0 = 42.435;
        */

        
        // *******************************************************
  
    
        // extract global position
        
        double lat = (navSbgMsg->position.x)*(M_PI/180); // [rad]
        double lon = (navSbgMsg->position.y)*(M_PI/180); // [rad]
        double h = (navSbgMsg->position.z + navSbgMsg->undulation); // [m] Height above WGS84 Ellipsoid = altitude + undulation
        

        // WGS-84 parameters
        double a = 6378137; // Semi-major axis (equitorial radius)
        double f = 1/298.257223563; // Flattening 
        double e = sqrt(2*f - pow(f,2)); // Earth eccentricity

        double dlon = lon - lon0;
        double dlat = lat - lat0;

        double Rn = a/sqrt(1 - pow(e,2)*pow(sin(lat0),2));
        double Rm = Rn * ((1 - pow(e,2)) / (1 - pow(e,2)*pow(sin(lat0),2)) );

        return vec3(dlat/atan2(1,Rm), dlon/atan2(1,Rn*cos(lat0)), h0 - h);
    }




    // convert from NED to WGS-84 (MSS toolbox)
    vec3 MessageHandler::flat2llh(vec3 llh_pos){

        // reference position parameters

        // *****************************************************

        // * experiment 3 - brattørkaia location

        // trinn 1
        
	    double lat0 = 1.10723500;
        double lon0 = 0.18151317;
        double h0 = 41.404;
        

         // trinn 2
        /*
        double lat0 = 1.10723515;
        double lon0 = 0.18151361;
        double h0 = 41.863; 
        */

        // trinn 3
        /*
        double lat0 = 1.10723531;
        double lon0 = 0.18151397;
        double h0 = 42.435;
        */

        
        // *******************************************************

        // WGS-84 parameters
        double a = 6378137; // Semi-major axis (equitorial radius)
        double f = 1/298.257223563; // Flattening 
        double e = sqrt(2*f - pow(f,2)); // Earth eccentricity

        double Rn = a/sqrt(1 - pow(e,2)*pow(sin(lat0),2));
        double Rm = Rn * ((1 - pow(e,2)) / (1 - pow(e,2)*pow(sin(lat0),2)) );

        double dlat = llh_pos.y() * atan2(1,Rm); // TODO: check order of x and y
        double dlon = llh_pos.x() * atan2(1,Rn*cos(lat0)); // TODO: check order of x and y

        
        double lon = mekf_.ssa(lon0 + dlon); // [rad] 
        double lat = mekf_.ssa(lat0 + dlat); // [rad] 
        double h = h0 - llh_pos.z(); // [m]

        return vec3(lat,lon,h);
            
    }

    
    


    

    // *** publisher - MEKF NAV ***
    void MessageHandler::publishEstimatedNav(const ros::TimerEvent&){

        if(mekf_.publish_MEKF_){

            // get mekf nav results
            vec3 pos = mekf_.getPosition();
            vec3 vel = mekf_.getVelocity();

            // transform position to WGS-84
            vec3 pos_llh = flat2llh(pos);

            // create SBG MEKF NAV msg
            sbg_driver::SbgEkfNav navSbgMsg;
            navSbgMsg.header.frame_id = "/sbg/mekf_nav";
            navSbgMsg.header.seq = trace_id_nav_++;
            navSbgMsg.header.stamp = ros::Time::now(); 

            navSbgMsg.position.x = pos_llh.x();
            navSbgMsg.position.y = pos_llh.y();
            navSbgMsg.position.z = pos_llh.z();

	        // TODO: SWITCH x and y velocity (since we switch x and y in flat2llh)
	        navSbgMsg.velocity.x = vel.y(); // swap
            navSbgMsg.velocity.y = vel.x(); // swap
            navSbgMsg.velocity.z = vel.z();
            
            // publish message
            pubEstimatedNav_.publish(navSbgMsg);

        }
        else
        {
            return;
        }
        

    }


    // *** publisher - MEKF EULER ***
    void MessageHandler::publishEstimatedEuler(const ros::TimerEvent&){

        if(mekf_.publish_MEKF_){

            // get mekf euler results
            quat quat_mekf = mekf_.getQuat();

            // convert to euler
            geometry_msgs::Quaternion quat_msg;
            quat_msg.x = quat_mekf.x();
            quat_msg.y = quat_mekf.y();
            quat_msg.z = quat_mekf.z();
            quat_msg.w = quat_mekf.w();

            // quat -> tf
            tf::Quaternion quatTf;
            tf::quaternionMsgToTF(quat_msg, quatTf);

            double roll, pitch, yaw;
            tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw);

            // create SBG MEKF EULER msg
            sbg_driver::SbgEkfEuler eulerSbgMsg;

            eulerSbgMsg.header.frame_id = "/sbg/mekf_euler";
            eulerSbgMsg.header.seq = trace_id_euler_++;
            eulerSbgMsg.header.stamp = ros::Time::now(); 

            eulerSbgMsg.angle.x = roll;
            eulerSbgMsg.angle.y = pitch;
            eulerSbgMsg.angle.z = yaw;

            // publish message 
            pubEstimatedEuler_.publish(eulerSbgMsg);


        }
        else
        {
            return;
        }


    }

    




}
