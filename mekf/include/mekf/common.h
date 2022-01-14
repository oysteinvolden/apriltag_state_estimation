#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <math.h>

namespace mekf{


    typedef float scalar_t;
    typedef Eigen::Matrix<double, 3, 1> vec3;  // Vector in R3
    typedef Eigen::Quaternion<double> quat;    // Member of S4   
    typedef Eigen::Matrix<double, 3, 3> mat3;  // Matrix in R3
   
    // State vector:
    // * Attitude quaternion
    // * NED velocity
    // * NED position
    // * Delta Angle bias - rad
    // * Delta Velocity bias - m/s

    struct state {
        quat  quat_nominal; // quaternion defining the rotaton from NED to XYZ frame
        vec3  vel;          // NED velocity in earth frame in m/s
        vec3  pos;          // NED position in earth frame in m
        vec3  gyro_bias;    // delta angle bias estimate in rad
        vec3  accel_bias;   // delta velocity bias estimate in m/s
    };

    struct imuSample {
        vec3    delta_ang;    // delta angle in body frame (integrated gyro measurements) (rad)
        vec3    delta_vel;    // delta velocity in body frame (integrated accelerometer measurements) (m/sec)
        double  delta_ang_dt; // delta angle integration period (sec)
        double  delta_vel_dt; // delta velocity integration period (sec)
        uint64_t  time_us;    // timestamp of the measurement (uSec)
    };

    struct cameraPoseSample {
        vec3 posNED;       // measured NED body position relative to the local origin (m)
        quat quatNED;      // measured quaternion orientation defining rotation from NED to body frame
        double posErr;     // 1-Sigma spherical position accuracy (m)
        double angErr;     // 1-Sigma angular error (rad)
        uint64_t time_us;  // timestamp of the measurement (micro seconds)
    };


    // skew-symmetric matrix (MSS toolbox)
    inline mat3 Smtrx(vec3 a){
         mat3 symMat;
         symMat << 0,-a(2), a(1),
                   a(2), 0, -a(0),
                  -a(1), a(0), 0;
         return symMat;
    }

    // quat to euler (MSS toolbox)
    inline vec3 q2euler(quat q){

        double phi, theta, psi;

        q.normalize();
        mat3 R = q.toRotationMatrix();

        phi = atan2(R(3,2),R(3,3));

        if(abs(R(3,1)) > 1){
            R(3,1) = signbit(R(3,1));
        }
        else
        {
            theta = -asin(R(3,1));
        }

        psi = atan2(R(2,1),R(1,1));
        
        return vec3(phi,theta,psi);

    }


    // *** tf functions ***

    inline quat from_axis_angle(const vec3 &axis, scalar_t theta) {
        quat q;

        if (theta < scalar_t(1e-10)) {
        q.w() = scalar_t(1.0);
        q.x() = q.y() = q.z() = 0;
        }

        scalar_t magnitude = sin(theta / 2.0f);

        q.w() = cos(theta / 2.0f);
        q.x() = axis(0) * magnitude;
        q.y() = axis(1) * magnitude;
        q.z() = axis(2) * magnitude;
        
        return q;
    }

    inline quat from_axis_angle(vec3 vec) {
        quat q;
        scalar_t theta = vec.norm();

        if (theta < scalar_t(1e-10)) {
            q.w() = scalar_t(1.0);
            q.x() = q.y() = q.z() = 0;
            return q;
        }

        vec3 tmp = vec / theta;
        return from_axis_angle(tmp, theta);
    }
    

    }



#endif /* defined(COMMON_H_) */