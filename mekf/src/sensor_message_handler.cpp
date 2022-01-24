#include <mekf/sensor_message_handler.h>


namespace mekf{

    MessageHandler::MessageHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
        


        ROS_INFO("Subscribing to IMU");

        subImu_ = nh_.subscribe("/zed2i/zed_node/imu/data", 1000, &MessageHandler::imuCallback, this);
        
        ROS_INFO("Subscribing to camera pose");
        
        subCameraPose_ = nh_.subscribe("/apriltag_bundle_pose", 1, &MessageHandler::cameraPoseCallback, this);

        pubEstimatedPose_ = nh_.advertise<nav_msgs::Odometry>("mekf_pose", 1);

        int publish_rate = publish_rate_;
        pubTimer_ = nh_.createTimer(ros::Duration(1.0f/publish_rate), &MessageHandler::publishState, this);

    }


    // -------------------------------------------------
    // %%% IMU %%%%
    // -------------------------------------------------


    // Rotate IMU to align with body frame (X forward, Y right, Z down)
    Eigen::Transform<double,3,Eigen::Affine> MessageHandler::getImuToBodyT(){

        // %%%% Part 1: from IMU to cam %%%%

        // left cam to imu transform - given by ZED API 
        Eigen::Quaternion<double> R_cam_to_imu(0.99999618, -0.00012207, -0.00272684, -0.00017263); 
        Eigen::Transform<double,3,Eigen::Affine> T_cam_to_imu(R_cam_to_imu);

        // from IMU to left cam (inverse) - we need this transformation since we know left cam frame is in ENU
        Eigen::Transform<double,3,Eigen::Affine> T_imu_to_cam = T_cam_to_imu.inverse();

        // %%%% part 2: from cam to body %%%%

        // roll, pitch, yaw - order: about X Y Z respectively 
        double roll=M_PI, pitch=0, yaw=0;
        Eigen::Quaternion<double> R_cam_to_body;
        R_cam_to_body = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());
        Eigen::Transform<double,3,Eigen::Affine> T_cam_to_body(R_cam_to_body);

        // %%% part 3: concatenate transformations -> imu to body
        
        Eigen::Transform<double,3,Eigen::Affine> T_imu_to_body = T_cam_to_body*T_imu_to_cam; // from right to left: imu -> cam -> body
   
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

    void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

        // Since ZED IMU run on 400 Hz whatsoever, we simply accept every fourth imu message to get approx 100 hz
        if((imuMsg->header.seq%4) == 0){

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

                // run kalman filter
                mekf_.run_mekf(ang_vel, lin_acc, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), dt);


            }

            prevStampImu_ = imuMsg->header.stamp;
        
        }
    }



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
        pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - 0.023;
        pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.002;
        pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z + 0.002; 
 
        // if we use static transform from camera to center of vehicle in camera frame instead %%% 
        // Measured offsets between camera and center of vehicle (in camera frame):
        // x = 0.06 m, y = 0.3115 m, z = 0.033 m 
   
        //pose.pose.pose.position.x = cameraPoseIn->pose.pose.position.x - 0.06;
        //pose.pose.pose.position.y = cameraPoseIn->pose.pose.position.y - 0.3115;
        //pose.pose.pose.position.z = cameraPoseIn->pose.pose.position.z - 0.033; 

        
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
        yaw_offset_2 = 1.5*(M_PI/180); // TODO: configure for step 1: , step 2: , step 3:  
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

            // update with pose sample
            mekf_.updateCamPose(cam_pos, cam_quat, static_cast<uint64_t>(cameraPoseMsg->header.stamp.toSec()*1e6f));
                        
        }

        prevStampCameraPose_ = cameraPoseMsg->header.stamp;
    }




    // TODO: update later
    void MessageHandler::publishState(const ros::TimerEvent&) {

        
        // get mekf results
        const quat e2g = mekf_.getQuat();
        const vec3 position = mekf_.getPosition();
        const vec3 velocity = mekf_.getVelocity();

        static size_t trace_id_ = 0;
        std_msgs::Header header;
        header.frame_id = "/pose_mekf";
        header.seq = trace_id_++;

        // we use imu time to compare with rosbags
        // use ros::Time::now() in real-time applications
        
        //ros::Time imu_time(mekf_.getImuTime()); 
        //header.stamp = imu_time; 
        header.stamp = ros::Time::now();

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

        pubEstimatedPose_.publish(odom);
        
    }



}
