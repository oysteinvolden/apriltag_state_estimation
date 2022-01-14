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
      
      static constexpr int publish_rate = 25; // TODO: check later
      
      MessageHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh); 


    private:
      
      ros::NodeHandle nh_;

      // publisher
      ros::Publisher pubEstimatedPose_;
      void publishState();

      // subscribers
      ros::Subscriber subImu_;
      ros::Subscriber subCameraPose_;

      // callbacks
      void imuCallback(const sensor_msgs::ImuConstPtr&);
      void cameraPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);

      // Camera pose transforms  
      geometry_msgs::PoseWithCovarianceStamped cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn);  
    
      // timing
      ros::Time prevStampImu_;
      ros::Time prevStampCameraPose_;

      mekf::MEKF mekf_;

      // initalization
      bool init_;



  };

}



#endif /* defined(MESSAGE_HANDLER_H_) */