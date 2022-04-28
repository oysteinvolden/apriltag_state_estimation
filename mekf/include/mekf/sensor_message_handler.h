#ifndef MESSAGE_HANDLER_HPP_
#define MESSAGE_HANDLER_HPP_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// ROS sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

#include <mekf/mekf.h>

// SBG
#include <mekf/sbg_driver/SbgImuData.h>
#include <mekf/sbg_driver/SbgEkfNav.h>
#include <mekf/sbg_driver/SbgEkfQuat.h>
#include <mekf/sbg_driver/SbgEkfEuler.h>




namespace mekf {

  class MessageHandler {
  
    public:
      
      static constexpr int publish_rate_ = 50;

      MessageHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh); 



    private:
      
      ros::NodeHandle nh_;

      // publisher
      ros::Publisher pubEstimatedNav_;
      ros::Publisher pubEstimatedEuler_;
      void publishEstimatedNav(const ros::TimerEvent&);
      void publishEstimatedEuler(const ros::TimerEvent&);


      // counter for seq number published 
      size_t trace_id_nav_ = 0;
      size_t trace_id_euler_ = 0;

      // subscribers
      ros::Subscriber subImu_;
      ros::Subscriber subCameraPose_;
      ros::Subscriber subEkfNav_;
      ros::Subscriber subEkfEuler_;

      // IMU transforms
      sensor_msgs::Imu imuTransform(const sensor_msgs::ImuConstPtr &imu_in, const Eigen::Transform<double,3,Eigen::Affine> &T);
      Eigen::Transform<double,3,Eigen::Affine> getImuToBodyT();

      // callbacks
      void imuCallback(const sensor_msgs::ImuConstPtr& imuMsg);
      void cameraPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& camMsg);
      void ekfNavCallback(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg); 
      void ekfEulerCallback(const sbg_driver::SbgEkfEulerConstPtr& eulerSbgMsg); 

      // Camera pose transforms  
      geometry_msgs::PoseWithCovarianceStamped cameraTransform(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn);  

      // convert from WGS-84 to NED (MSS toolbox)
      vec3 llh2flat(const sbg_driver::SbgEkfNavConstPtr& navSbgMsg);

      // convert from NED to WGS-84 (MSS toolbox)
      vec3 flat2llh(vec3 llh_pos);
    
      // timing
      ros::Time prevStampImu_;
      ros::Time prevStampCameraPose_; 
      ros::Time prevStampSbgEkfNav_;
      ros::Time prevStampSbgEkfEuler_;

      //ros::Timer pubTimer_;
      ros::Timer pubTimerNav_; // for create timer
      ros::Timer pubTimerEuler_; // for create timer

      mekf::MEKF mekf_;

      // initalization
      bool init_;



  };

}



#endif /* defined(MESSAGE_HANDLER_H_) */
