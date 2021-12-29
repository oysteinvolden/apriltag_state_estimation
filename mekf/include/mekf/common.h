#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace mekf{


    typedef Eigen::Matrix<double, 3, 1> vec3; 
    typedef Eigen::Quaternion<double> quat;  
   
    // State vector:
    // * Attitude quaternion
    // * NED velocity
    // * NED position
    // * Delta Angle bias - rad
    // * Delta Velocity bias - m/s

    struct state {
        quat  quat_nominal; ///< quaternion defining the rotaton from NED to XYZ frame
        vec3  vel;          ///< NED velocity in earth frame in m/s
        vec3  pos;          ///< NED position in earth frame in m
        vec3  gyro_bias;    ///< delta angle bias estimate in rad
        vec3  accel_bias;   ///< delta velocity bias estimate in m/s
    };

    struct imuSample {
        vec3    delta_ang;      ///< delta angle in body frame (integrated gyro measurements) (rad)
        vec3    delta_vel;      ///< delta velocity in body frame (integrated accelerometer measurements) (m/sec)
        double  delta_ang_dt; ///< delta angle integration period (sec)
        double  delta_vel_dt; ///< delta velocity integration period (sec)
        uint64_t  time_us;      ///< timestamp of the measurement (uSec)
    };

    struct cameraPoseSample {
        vec3 posNED;       ///< measured NED body position relative to the local origin (m)
        quat quatNED;      ///< measured quaternion orientation defining rotation from NED to body frame
        double posErr;   ///< 1-Sigma spherical position accuracy (m)
        double angErr;   ///< 1-Sigma angular error (rad)
        uint64_t time_us;  ///< timestamp of the measurement (micro seconds)
    };
  

}

#endif /* defined(COMMON_H_) */