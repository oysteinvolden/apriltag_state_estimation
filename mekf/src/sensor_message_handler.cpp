#include <mekf/sensor_message_handler.h>

// libraries for file streams
#include <iostream>
#include <fstream>
#include <sstream>

// text files to log sbg data
//const char *path_log_mekfApriltag="/home/oysteinvolden/mekf_ws3/logging/2021-11-09-16-27-57/mekf/mekf_apriltag.txt";
//const char *path_log_mekfApriltag="/home/oysteinvolden/mekf_ws3/logging/2021-11-09-16-25-43/mekf/mekf_apriltag.txt";
//const char *path_log_mekfApriltag="/home/oysteinvolden/mekf_ws3/logging/2021-11-08-14-54-21/mekf/mekf_apriltag.txt";
const char *path_log_mekfApriltag="/home/oysteinvolden/mekf_ws3/logging/2021-11-09-13-58-55/mekf/mekf_apriltag.txt";
//const char *path_log_mekfApriltag="/home/oysteinvolden/mekf_ws3/logging/2021-11-09-16-29-44/mekf/mekf_apriltag.txt";
std::ofstream log_mekfApriltag(path_log_mekfApriltag);


namespace mekf{

    MessageHandler::MessageHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
        


        ROS_INFO("Subscribing to IMU");

        //subImu_ = nh_.subscribe("/sbg/imu_data", 1, &MessageHandler::imuCallback, this); // SBG IMU
        subImu_ = nh_.subscribe("/adis_imu", 10, &MessageHandler::imuCallback, this); // ADIS IMU
        
        ROS_INFO("Subscribing to camera pose");
        
        subCameraPose_ = nh_.subscribe("/apriltag_bundle_pose", 1, &MessageHandler::cameraPoseCallback, this);

        ROS_INFO("Subscribing to SBG EKF NAV and SBG EKF QUAT");

        // *** SBG ***

        // SBG EKF NAV
        subEkfNav_ = nh_.subscribe("/sbg/ekf_nav", 10, &MessageHandler::ekfNavCallback, this);

        // SBG EKF Euler
        subEkfEuler_ = nh_.subscribe("/sbg/ekf_euler", 10, &MessageHandler::ekfEulerCallback, this);
        
        
        
        /*
        message_filters::Subscriber<sbg_driver::SbgEkfNav> nav_sbg(nh_, "/sbg/ekf_nav", 1);
        message_filters::Subscriber<sbg_driver::SbgEkfEuler> euler_sbg(nh_, "/sbg/ekf_euler", 1);
  
        typedef message_filters::sync_policies::ApproximateTime<sbg_driver::SbgEkfNav, sbg_driver::SbgEkfEuler> MySyncPolicy;

        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), nav_sbg, euler_sbg);
        sync.registerCallback(boost::bind(&MessageHandler::sbgEkfCallback, this, _1, _2));
        */


        pubEstimatedPose_ = nh_.advertise<nav_msgs::Odometry>("mekf_pose", 1);

        int publish_rate = publish_rate_;
        pubTimer_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &MessageHandler::publishState, this);

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
    

    
    // OBS! we only transform linear acceleration and angular velocity
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
    // * Reduce frequency from 400 Hz to 100 Hz 
    // * Transform imu data to body frame
    // * run kalman filter with imu data
    // ------------------------------


    // EDIT: we use ADIS imu instead of SBG imu
    
    void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

        // ADIS imu run on 250 Hz -> downsample to 125 Hz

        if((imuMsg->header.seq%5) == 0){

            if(prevStampImu_.sec > 0){

                    
                if(!init_){
                        init_ = true;
                        ROS_INFO("Initialized MEKF");
                }   
                
                // delta time
                double dt = (imuMsg->header.stamp - prevStampImu_).toSec(); // TODO: only neccessary if we don't use fixed sampling time (h)

                
                // imu to body transformation
                Eigen::Transform<double,3,Eigen::Affine> imuToBodyT = getImuToBodyT();
    
                // transform imu data to body frame
                sensor_msgs::Imu imuInBody = imuTransform(imuMsg, imuToBodyT);

                // get measurements
                vec3 ang_vel = vec3(imuInBody.angular_velocity.x, imuInBody.angular_velocity.y, imuInBody.angular_velocity.z);
                vec3 lin_acc = vec3(imuInBody.linear_acceleration.x, imuInBody.linear_acceleration.y, imuInBody.linear_acceleration.z);
                

                //vec3 ang_vel = vec3(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
                //vec3 lin_acc = vec3(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);


                //std::cout << "ang vel: " << ang_vel << std::endl;
                //std::cout << "lin acc: " << std::endl;
                //std::cout << lin_acc << std::endl;

                // run kalman filter
                mekf_.run_mekf(ang_vel, lin_acc, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), dt);

            
            }

            prevStampImu_ = imuMsg->header.stamp;
        }

    }
    
    


    // EDIT: custom sbg imu instead of ROS imu
    /*
    void MessageHandler::imuCallback(const sbg_driver::SbgImuDataConstPtr& imuMsg){

        // SBG imu data run 25 Hz, potentially 100 hz


        if(prevStampImu_.sec > 0){

                
            if(!init_){
                    init_ = true;
                    ROS_INFO("Initialized MEKF");
            }   
            
            // delta time
            double dt = (imuMsg->header.stamp - prevStampImu_).toSec(); // TODO: only neccessary if we don't use fixed sampling time (h)


            // get measurements
            vec3 ang_vel = vec3(imuMsg->gyro.x, imuMsg->gyro.y, imuMsg->gyro.z);
            vec3 lin_acc = vec3(imuMsg->accel.x, imuMsg->accel.y, imuMsg->accel.z);

            //std::cout << "ang vel: " << ang_vel << std::endl;
            //std::cout << "lin acc: " << lin_acc << std::endl;

            // run kalman filter
            mekf_.run_mekf(ang_vel, lin_acc, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), dt);

        
        }

        prevStampImu_ = imuMsg->header.stamp;

    }
    */
    



    // -------------------------------------------------
    // %%% Camera %%%%
    // -------------------------------------------------


    geometry_msgs::PoseWithCovarianceStamped MessageHandler::cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn){


        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header = cameraPoseIn->header;


        // %%% Part 1: BODY transformations 
        // Add "cam to imu" translation offset (in camera optical frame) to the "cam to tag pose" %%%
        // * Optical camera frame: X right, Y down, Z forward.
        // * Hence, camera to tag pose is expressed in left camera optical frame
        // * and we move it to imu frame by a static "cam to imu" translation offset (no rotation).
        // * NB! Cam to imu offset is given by ZED Sterelabs API.
   
        // TODO: doublecheck signs
        //pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - 0.023;
        //pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.002;
        //pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z + 0.002; 
 
        // if we use static transform from camera to center of vehicle in camera frame instead %%% 
        // Measured offsets between camera and center of vehicle (in camera frame):
        // x = 0.06 m, y = 0.3115 m, z = 0.033 m 
   
        // left camera (default)
        /*
        pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - (0.06); // TODO: check
        pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.3115; 
        pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z - (0.033); // TODO:check 
        */

        // right camera (experiment 2 - rain)
        pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x + (0.06); // right camera
        pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.3115; 
        pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z - (0.033); 

        /*
        pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - (0.06 + 0.15); // TODO: check
        pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.3115; 
        pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z - (0.033 + 0.07); // TODO:check 
        */
        
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
        // * 1. A fixed, measured rotation about Z axis (yaw): 227 degrees
        // * 2. A small rotation about Z axis (yaw) depending on which marker configuration is used (between 1.5 and 2.5 degrees)
        // * NB! The small rotation in (2) above is found by subtracting for the constant heading offset between ground truth SBG Ellipse INS and Apriltag
        // * The same small rotation is tested for multiple scenarios to show reproducibility -> E.g., 227 deg + 1.5 deg is closer to the true yaw offset 

        // pose rotated NED (to align apriltag with true north) 
        geometry_msgs::PoseWithCovarianceStamped pose_NED_rot;
        pose_NED_rot.header = cameraPoseIn->header; // copy header from original incoming message

        float yaw_offset, yaw_offset_1, yaw_offset_2;
        yaw_offset_1 = 227*(M_PI/180);
        //yaw_offset_2 = 1.5*(M_PI/180); // TODO: configure for step 1: , step 2: , step 3:  
        yaw_offset_2 = 1*(M_PI/180); // TODO: configure for step 1: , step 2: , step 3:  
        yaw_offset = yaw_offset_1 + yaw_offset_2;


        // %%% 4.1: find heading relative to true North %%%

        // * NB! Given that we have the tag frame as specified by AprilTag and rotate it by q_rot_1 to get the pose_NED orientation,
        // * we only need to rotate about z axis to find heading relative to true north.
        // * Hence, we do the following to find the NED heading angle:
        // * Subtract the measured Apriltag yaw and 90 degrees from the yaw offset (227 + 1.5 deg)
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

            // delta time 
            const double dt = (cameraPoseMsg->header.stamp - prevStampCameraPose_).toSec(); // TODO: neccessary?

            // transform camera pose to NED
            geometry_msgs::PoseWithCovarianceStamped camPoseNED = cameraTransform(cameraPoseMsg);
   
            // get measurements
            vec3 cam_pos = vec3(camPoseNED.pose.pose.position.x, camPoseNED.pose.pose.position.y, camPoseNED.pose.pose.position.z);
            quat cam_quat = quat(camPoseNED.pose.pose.orientation.w, camPoseNED.pose.pose.orientation.x, camPoseNED.pose.pose.orientation.y, camPoseNED.pose.pose.orientation.z);

            // update pose sample
            mekf_.updateCamPose(cam_pos, cam_quat, static_cast<uint64_t>(cameraPoseMsg->header.stamp.toSec()*1e6f));
                        
        }

        prevStampCameraPose_ = cameraPoseMsg->header.stamp;
    }



    void MessageHandler::ekfNavCallback(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg){

        if(prevStampSbgEkfNav_.sec > 0){

            // convert from WGS84 to NED
            vec3 sbgPosNED = ll2flat(navSbgMsg);
            vec3 sbg_pos = vec3(sbgPosNED.y(), sbgPosNED.x(), sbgPosNED.z()); // TODO: check this - order of x and y

            //std::cout << "sbg pos: " << sbg_pos << std::endl;

            // update sample
            mekf_.updateSbgPos(sbg_pos, static_cast<uint64_t>(navSbgMsg->header.stamp.toSec()*1e6f));
        }

        prevStampSbgEkfNav_ = navSbgMsg->header.stamp;

    }

    void MessageHandler::ekfEulerCallback(const sbg_driver::SbgEkfEulerConstPtr& eulerSbgMsg){

        if(prevStampSbgEkfEuler_.sec > 0){

            // convert from euler to quat
            vec3 sbg_euler = vec3(eulerSbgMsg->angle.x, eulerSbgMsg->angle.y, eulerSbgMsg->angle.z);
            quat sbg_quat = euler2q(sbg_euler);

            //std::cout << "sbg quat: " << sbg_quat.w() << " " << sbg_quat.x() << " " << sbg_quat.y() << " " << sbg_quat.z() << std::endl;

            // update pose sample
            mekf_.updateSbgQuat(sbg_quat, static_cast<uint64_t>(eulerSbgMsg->header.stamp.toSec()*1e6f));
        }

        prevStampSbgEkfEuler_ = eulerSbgMsg->header.stamp;
        

    }



    // *** SBG INS Callback ***
    // -------------------------
    // * Take in SBG EKF NAV and SBG EKF QUAT messages
    // * Convert SBG EKF NAV from WGS84 to local NED frame
    // * Push data to SBG struct

    /*
    void MessageHandler::sbgEkfCallback(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg, const sbg_driver::SbgEkfEulerConstPtr& eulerSbgMsg){

        std::cout << "test SBG" << std::endl;

        if(prevStampSbg_.sec > 0){

            // convert from WGS84 to NED
            vec3 sbgPosNED = ll2flat(navSbgMsg);
            vec3 sbg_pos = vec3(sbgPosNED.x(), sbgPosNED.y(), sbgPosNED.z());

            // convert from euler to quat
            vec3 sbg_euler = vec3(eulerSbgMsg->angle.x, eulerSbgMsg->angle.y, eulerSbgMsg->angle.z);
            quat sbg_quat = euler2q(sbg_euler);

            // update pose sample
            mekf_.updateSbgPose(sbg_pos, sbg_quat, static_cast<uint64_t>(navSbgMsg->header.stamp.toSec()*1e6f));
        }

        prevStampSbg_ = navSbgMsg->header.stamp;

    }
    */
    

    
    // convert from WGS-84 to NED (MSS toolbox)
    vec3 MessageHandler::ll2flat(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg){

        // reference parameters
        // * step 2 (day 2)
        
        /*
        double lat0 = 1.10723534;
        double lon0 = 0.18151399;
        double h0 = 42.08; 
        */

        // experiment 1
        // day 1 - step 3
        
        /*
        double lat0 = 1.10723547;
        double lon0 = 0.18151444;
        double h0 = 42.98; 
        */

        //double lat0 = 1.10723554 - 0.00000008;
        //double lon0 = 0.18151445 - 0.0000001;
        //double h0 = 42.604;

        // day 2 - step 3
        //double lat0 = 1.10723554 - 0.000000036;// works for 21-45
        //double lon0 = 0.18151445 - 0.00000008; // works for 21-45
        //double h0 = 42.604;

        /*
        double lat0 = 1.10723554 + 0.00000001; // 51-20 - nja?
        double lon0 = 0.18151445 - 0.00000003; // 51-20 - nja?
        double h0 = 42.604;
        */

        
        double lat0 = 1.10723554;// works for 48-27 / 51-20
        double lon0 = 0.18151445; // works for 48-27 51-20
        double h0 = 42.604;
        

        //double lat0 = 1.10723554; // works for 2021-11-09-14-55-37
        //double lon0 = 0.18151445 - 0.00000002; // works for 2021-11-09-14-55-37
        //double h0 = 42.604;
        

        // day 1 - step 3 (alternative 2)
        /*
        double lat0 = 1.10723554;
        double lon0 = 0.18151445;
        double h0 = 42.604; 
        */
     
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
    

      


    void MessageHandler::publishState(const ros::TimerEvent&){


        // TODO: based on flag, use sbg directly or estimated state. We only use estimated state when its initialized

        
        // get mekf results
        const quat e2g = mekf_.getQuat();
        const vec3 position = mekf_.getPosition();
        const vec3 velocity = mekf_.getVelocity();

        static size_t trace_id_ = 0;
        std_msgs::Header header;
        header.frame_id = "/pose_mekf";
        header.seq = trace_id_++;


        // TODO: transform back to WGS84 for feedback experiment?

        // we use imu time to compare with rosbags
        // TODO: use ros::Time::now() in real-time applications?
        // or imu time + processing delay
        
        //ros::Time imu_time(mekf_.getImuTime()); 
        //header.stamp = imu_time; 
        header.stamp = ros::Time::now();

        // for logging, TODO: fix later
        uint64_t time_stamp_imu = mekf_.getImuTime();

        nav_msgs::Odometry odom;
        odom.header = header;
        odom.pose.pose.position.x = position[0]; 
        odom.pose.pose.position.y = position[1];
        odom.pose.pose.position.z = position[2];
        odom.twist.twist.linear.x = velocity[0];
        odom.twist.twist.linear.y = velocity[1];
        odom.twist.twist.linear.z = velocity[2];
        odom.pose.pose.orientation.w = e2g.w();
        odom.pose.pose.orientation.x = e2g.x();
        odom.pose.pose.orientation.y = e2g.y();
        odom.pose.pose.orientation.z = e2g.z();


        // %%% inverse %%%
        /*
        // pose -> tf
        tf::Stamped<tf::Transform> tag_transform;
        tf::poseMsgToTF(odom.pose.pose, tag_transform);

        // tf -> tf inverse
        tf::Transform tag_transform_inverse;
        tag_transform_inverse = tag_transform.inverse();

        // tf inverse -> pose inverse
        geometry_msgs::PoseWithCovarianceStamped odom_inverse;
        odom_inverse.header = header;
        tf::poseTFToMsg(tag_transform_inverse, odom_inverse.pose.pose);


        // %%% transform -> sbg -> center of vehicle in body frame
        odom_inverse.pose.pose.position.x = odom_inverse.pose.pose.position.x - 0.07; // TODO: check 
        odom_inverse.pose.pose.position.y = odom_inverse.pose.pose.position.y - 0.15; // TODO: check

        // %% inverse again %%

        // pose -> tf
        tf::Stamped<tf::Transform> tag_transform2;
        tf::poseMsgToTF(odom_inverse.pose.pose, tag_transform2);

        // tf -> tf inverse
        tf::Transform tag_transform_inverse2;
        tag_transform_inverse2 = tag_transform2.inverse();

        // tf inverse -> pose inverse
        geometry_msgs::PoseWithCovarianceStamped odom_recovered;
        odom_recovered.header = header;
        tf::poseTFToMsg(tag_transform_inverse2, odom_recovered.pose.pose);


        */
        // %%%%%%%%%%%%%%%%%%%%%%%%

        //pubEstimatedPose_.publish(odom_recovered);
        pubEstimatedPose_.publish(odom);



        // convert quat to euler, TODO: use quat2euler instead?
        geometry_msgs::Quaternion quat_msg;

        quat_msg.x = e2g.x();
	    quat_msg.y = e2g.y();
	    quat_msg.z = e2g.z();
	    quat_msg.w = e2g.w();

	    // quat -> tf
	    tf::Quaternion quat_estimated;
	    tf::quaternionMsgToTF(quat_msg, quat_estimated);

        double roll_est, pitch_est, yaw_est;
        tf::Matrix3x3(quat_estimated).getRPY(roll_est, pitch_est, yaw_est); 


        // write results to text file
        log_mekfApriltag << std::fixed;
        log_mekfApriltag << std::setprecision(16);
        //log_mekfApriltag << header.seq << " " << time_stamp_imu*10e-7 << " " << odom_recovered.pose.pose.position.x << " " << odom_recovered.pose.pose.position.y << " " << odom_recovered.pose.pose.position.z << " " << yaw_est << std::endl;
        log_mekfApriltag << header.seq << " " << time_stamp_imu*10e-7 << " " << position[0] << " " << position[1] << " " << position[2] << " " << yaw_est << std::endl;

        if(trace_id_ > 24500){
            log_mekfApriltag.close();
        }
        
    }




}
