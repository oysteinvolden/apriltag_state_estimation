#ifndef MESSAGE_HANDLER_HPP_
#define MESSAGE_HANDLER_HPP_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <mekf/mekf.h>





namespace mekf {

  class MessageHandler {
  
    public:
      
      static constexpr int publish_rate_ = 100; 
      
      MessageHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh); 


    private:
      
      ros::NodeHandle nh_;

      // publisher
      ros::Publisher pubEstimatedPose_;
      void publishState(const ros::TimerEvent&);

      // subscribers
      ros::Subscriber subImu_;
      ros::Subscriber subCameraPose_;

      // callbacks
      void imuCallback(const sensor_msgs::ImuConstPtr&);
      void cameraPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

      // IMU transforms
      sensor_msgs::Imu imuTransform(const sensor_msgs::ImuConstPtr &imu_in, const Eigen::Transform<double,3,Eigen::Affine> &T);
      Eigen::Transform<double,3,Eigen::Affine> getImuToBodyT();

      // Camera pose transforms  
      geometry_msgs::PoseWithCovarianceStamped cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn);  

      // Move estimated position from imu to center of vehicle after fusion
      Eigen::Transform<double,3,Eigen::Affine> positionTransform(quat quat_in, vec3 pos_in);
    
      // timing
      ros::Time prevStampImu_;
      ros::Time prevStampCameraPose_;
      ros::Timer pubTimer_;

      mekf::MEKF mekf_;

      // initalization
      bool init_;



  };

}



#endif /* defined(MESSAGE_HANDLER_H_) */