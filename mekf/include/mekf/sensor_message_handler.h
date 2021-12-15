// ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Geometry>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

//#include <mekf/MEKF.hpp>

namespace mekf {

  class MessageHandler {
  
    public:
      
      static constexpr int publish_rate = 100;
      
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

      // IMU transforms
      sensor_msgs::Imu imuTransform(const sensor_msgs::ImuConstPtr &imu_in, const Eigen::Transform<double,3,Eigen::Affine> &T);
      Eigen::Transform<double,3,Eigen::Affine> getImuToBodyT();

      // Camera pose transforms

      // eigen version - not correct yet
      geometry_msgs::PoseWithCovarianceStamped cameraTransformEigen(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn);
      // ROS version - works fine
      geometry_msgs::PoseWithCovarianceStamped cameraTransformROS(const geometry_msgs::PoseWithCovarianceStampedConstPtr& cameraPoseIn);

      // timing
      ros::Time prevStampImu_;
      ros::Time prevStampCameraPose_;


  };

}// namespace mekf
