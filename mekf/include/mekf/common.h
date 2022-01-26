#ifndef COMMON_H_
#define COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <unsupported/Eigen/MatrixFunctions> // used for matrix exponential

#include <cmath>
#include <math.h>




namespace mekf{


    typedef float scalar_t;
    typedef Eigen::Matrix<double, 3, 1> vec3;  // Vector in R3
    typedef Eigen::Matrix<double, 4, 1> vec4;  // Vector in R3
    typedef Eigen::Quaternion<double> quat;    // Member of S4   
    typedef Eigen::Matrix<double, 3, 3> mat3;  // Matrix in R3
    typedef Eigen::Matrix<double, 4, 4> mat4;   // Matrix in R3
   
    // State vector:
    // * Attitude quaternion
    // * NED velocity
    // * NED position
    // * Delta Angle bias - rad
    // * Delta Velocity bias - m/s

    struct state {
        quat  quat_nominal; // quaternion defining the rotaton from NED to XYZ frame
        vec3  vel;          // NED velocity in earth frame [m/s]
        vec3  pos;          // NED position in earth frame [m]
        vec3  gyro_bias;    // delta angle bias estimate [rad]
        vec3  accel_bias;   // delta velocity bias estimate [m/s]
    };

    struct imuSample {
        vec3    delta_ang;    // delta angle in body frame (integrated gyro measurements) [rad]
        vec3    delta_vel;    // delta velocity in body frame (integrated accelerometer measurements) [m/sec)]
        double  delta_ang_dt; // delta angle integration period [sec]
        double  delta_vel_dt; // delta velocity integration period [sec]
        uint64_t  time_us;    // timestamp of the measurement [uSec]
    };

    struct cameraPoseSample {
        vec3 posNED;       // measured NED body position relative to the local origin [m]
        quat quatNED;      // measured quaternion orientation defining rotation from NED to body frame
        double posErr;     // 1-Sigma spherical position accuracy [m]
        double angErr;     // 1-Sigma angular error [rad]
        uint64_t time_us;  // timestamp of the measurement [micro seconds]
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

        phi = atan2(R.coeff(3,2),R.coeff(3,3));

        if(abs(R.coeff(3,1)) > 1){
            R(3,1) = signbit(R.coeff(3,1));
        }
        else
        {
            theta = -asin(R.coeff(3,1));
        }

        psi = atan2(R.coeff(2,1),R.coeff(1,1));
        
        return vec3(phi,theta,psi);

    }

    // cross product
    inline vec3 cross(vec3 a, vec3 b){
        return vec3(a.x()*b.x(),a.y()*b.y(),a.z()*b.z());
    }


    // Schur product (MSS toolbox)
    inline quat quatprod(quat q1, quat q2){
 
        double eta1 = q1.w();
        vec3 eps1 = vec3(q1.x(),q1.y(),q1.z());
        double eta2 = q2.w();
        vec3 eps2 = vec3(q2.x(),q2.y(),q2.z());
        
        double a = eta1*eta2 - eps1.transpose()*eps2;
        vec3 b = eta2*eps1 + eta1*eps2 + cross(eps1,eps2);

        return quat(a,b.x(),b.y(),b.z());     
    }


    
    // Tw = Tquat(w) computes the quaternion transformation matrix Tw of
    // dimension 4 x 4 for attitude such that q_dot = Tw * q
    // (MSS toolbox)
    
    inline mat4 Tquat(vec3 u){

        // assume angular rates of dimension 3 is used
        vec3 w = vec3(u.x(),u.y(),u.z());

        mat4 T;
        T(0,0) = 0;
        T.block(0,1,1,3) = -w.transpose();
        T.block(1,0,3,1) = w;
        T.block(1,1,3,3) = -Smtrx(w);

        return 0.5*T;
    }


    inline mat3 Rquat(quat q){

        double tol = 1e-6;

        if( abs( q.norm() - 1 ) > tol ){
            std::cout << "norm(q) must be equal to 1" << std::endl;
        }

        double eta = q.w();
        vec3 eps = vec3(q.x(),q.y(),q.z());

        mat3 S = Smtrx(eps);

        mat3 I_33;
        I_33.setIdentity();

        return I_33 + 2*eta*S + 2*S*S;

    }

  

    // Matrix exponential
    
    //inline mat4 expm(mat4 X){

        // extract eigenvalues/eigenvectors
        /*
        Eigen::EigenSolver<Eigen::Matrix4d> es(X);
        Eigen::MatrixXcd D = es.eigenvalues();
        Eigen::MatrixXcd V = es.eigenvectors();
        */
 
    //    return X.exp(); // return matrix exponential
    //}
    

      

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