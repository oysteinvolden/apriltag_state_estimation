#include <mekf/sensor_message_handler.h>
//#include <geometry_msgs/Vector3.h>

#include <cmath>

namespace mekf{

    MessageHandler::MessageHandler(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : nh_(pnh){
        
        ROS_INFO("Subscribing to IMU");

        subImu_ = nh_.subscribe("/zed2i/zed_node/imu/data", 1000, &MessageHandler::imuCallback, this);
        
        ROS_INFO("Subscribing to camera pose");
        
        subCameraPose_ = nh_.subscribe("/apriltag_bundle_pose", 1, &MessageHandler::cameraPoseCallback, this);

        //pubEstimatedPose_ = nh_.advertise<nav_msgs::Odometry>("estimated_pose", 1);

    }

    // Rotate IMU to align with body frame
    Eigen::Transform<double,3,Eigen::Affine> MessageHandler::getImuToBodyT(){

        // %%%% Part 1: from IMU to cam %%%%

        // left cam to imu transform - given from zed2i camera 
        Eigen::Quaternion<double> R_cam_to_imu(0.99999618, -0.00012207, -0.00272684, -0.00017263); 
        //Eigen::Translation<double,3> t_cam_to_imu(-0.002, -0.023, -0.002); 
        //Eigen::Transform<double,3,Eigen::Affine> T_cam_to_imu(t_cam_to_imu*R_cam_to_imu);
        Eigen::Transform<double,3,Eigen::Affine> T_cam_to_imu(R_cam_to_imu);

        // from IMU to left cam (inverse) - we need this transformation since we know left cam frame is in ENU
        Eigen::Transform<double,3,Eigen::Affine> T_imu_to_cam = T_cam_to_imu.inverse();

        // %%%% part 2: from cam to body %%%%

        // roll, pitch, yaw - order: about X Y Z respectively 
        double roll=M_PI, pitch=0, yaw=0;
        Eigen::Quaternion<double> R_cam_to_body;
        R_cam_to_body = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());
        //Eigen::Translation<double,3> t_cam_to_body(-0.0415,-0.06,-0.3215); // (assume rotated to body here, X forward, Y right, Z down)
        //Eigen::Translation<double,3> t_cam_to_body(0.0415,0.06,0.3215);
        
        //Eigen::Transform<double,3,Eigen::Affine> T_cam_to_body(t_cam_to_body*R_cam_to_body);
        Eigen::Transform<double,3,Eigen::Affine> T_cam_to_body(R_cam_to_body);

        // %%% part 3: concatenate transformations -> imu to body
        
        Eigen::Transform<double,3,Eigen::Affine> T_imu_to_body = T_cam_to_body*T_imu_to_cam; // from right to left: imu -> cam -> body
   
        //Eigen::Transform<double,3,Eigen::Affine> T_imu_to_body = T_imu_to_cam*T_cam_to_body;

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

        std::cout << "ang vel:" << std::endl;
        std::cout << imu_out.angular_velocity << std::endl;

        // linear acceleration
        Eigen::Vector3d acc = T * Eigen::Vector3d(imu_in->linear_acceleration.x, imu_in->linear_acceleration.y, imu_in->linear_acceleration.z);

        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        std::cout << "lin acc: " << std::endl;
        std::cout << imu_out.linear_acceleration << std::endl;


        return imu_out;
        
    }

    void MessageHandler::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

        if(prevStampImu_.sec > 0){

            // imu to NED transformation
            Eigen::Transform<double,3,Eigen::Affine> imuToBodyT = getImuToBodyT();
    
            // get imu data expressed in body frame
            sensor_msgs::Imu imu_in_body = imuTransform(imuMsg, imuToBodyT);
            
            // delta time
            const double delta = (imuMsg->header.stamp - prevStampImu_).toSec();

            // get measurements
            geometry_msgs::Vector3 ang_vel, lin_acc;

            // angular velocity
            ang_vel.x = imuMsg->angular_velocity.x;
            ang_vel.y = imuMsg->angular_velocity.y;
            ang_vel.z = imuMsg->angular_velocity.z;

            // linear acceleration
            lin_acc.x = imuMsg->linear_acceleration.x;
            lin_acc.y = imuMsg->linear_acceleration.y;
            lin_acc.z = imuMsg->linear_acceleration.z;
            
            // update mekf

        }

        prevStampImu_ = imuMsg->header.stamp;

    }


    // NB! camera pose is converted to NED in apriltag node, so we do not need to transform frame 

    void MessageHandler::cameraPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseMsg){

        if(prevStampCameraPose_.sec > 0){

            // delta time 
            const double delta = (cameraPoseMsg->header.stamp - prevStampCameraPose_).toSec();
            
            // get measurements
            geometry_msgs::Vector3 pos_cam;
            geometry_msgs::Quaternion quat_cam;

            // position
            pos_cam.x = cameraPoseMsg->pose.pose.position.x;
            pos_cam.y = cameraPoseMsg->pose.pose.position.y;
            pos_cam.z = cameraPoseMsg->pose.pose.position.z;
            
            // quaternion
            quat_cam.x = cameraPoseMsg->pose.pose.orientation.x;
            quat_cam.y = cameraPoseMsg->pose.pose.orientation.y;
            quat_cam.z = cameraPoseMsg->pose.pose.orientation.z;
            quat_cam.w = cameraPoseMsg->pose.pose.orientation.w;
    
            
            // update mekf
       
        }

        prevStampCameraPose_ = cameraPoseMsg->header.stamp;
    }




    // TODO: update when mekf is ready
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