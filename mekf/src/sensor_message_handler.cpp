#include <mekf/sensor_message_handler.h>


namespace mekf{

    MessageHandler::MessageHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh), init_(false) {
        


        ROS_INFO("Subscribing to IMU");

        subImu_ = nh_.subscribe("/zed2i/zed_node/imu/data", 1000, &MessageHandler::imuCallback, this);
        
        ROS_INFO("Subscribing to camera pose");
        
        subCameraPose_ = nh_.subscribe("/apriltag_bundle_pose", 1, &MessageHandler::cameraPoseCallback, this);

        //pubEstimatedPose_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);


    }


    // -------------------------------------------------
    // %%% IMU %%%%
    // -------------------------------------------------



    
    // *** IMU Callback ***
    // -----------------------------
    // * Take in imu messages 
    // * Assumes they are in body (X forward, Y right, Z down)
    // * Extract neccesary data
    // * run kalman filter with imu data
    // ------------------------------

    void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

        if(prevStampImu_.sec > 0){

            if(!init_){
                init_ = true;
                ROS_INFO("Initialized MEKF");
            }
          
            // delta time
            const double dt = (imuMsg->header.stamp - prevStampImu_).toSec();

            // get measurements
            vec3 ang_vel = vec3(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z);
            vec3 lin_acc = vec3(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z);

            // run kalman filter
            mekf_.run_mekf(ang_vel, lin_acc, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), dt);

        }

        prevStampImu_ = imuMsg->header.stamp;

    }



    // -------------------------------------------------
    // %%% Camera %%%%
    // -------------------------------------------------


    geometry_msgs::PoseWithCovarianceStamped MessageHandler::cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn){


        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header = cameraPoseIn->header;

        
        // %%% Part 1: static transform from camera to center of vehicle in camera frame %%% 
        
        // Measured offsets between camera and center of vehicle (in camera frame):
        // x = 0.06 m, y = 0.3115 m, z = 0.033 m 
   
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
            const double dt = (cameraPoseMsg->header.stamp - prevStampCameraPose_).toSec(); // neccessary here?

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
    void MessageHandler::publishState() {

        /*
        // get mekf results
        const quat e2g = mekf_.getQuat();
        const vec3 position = mekf_.getPosition();
        const vec3 velocity = mekf_.getVelocity();

        static size_t trace_id_ = 0;
        std_msgs::Header header;
        header.frame_id = "/pose_mekf";
        header.seq = trace_id_++;
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
        */
    }



}
