/***************************************************************************************************************************
* math_utils.h
*
* Author: Longhao Qian
*
* Update Time: 2019.9.23
*
* Introduction:  math utils functions 
*
*               1、转换 ref to https://github.com/PX4/Matrix/blob/56b069956da141da244926ed7000e89b2ba6c731/matrix/Euler.hpp
***************************************************************************************************************************/
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

using namespace std;

namespace math_utils{

    enum vector_component{
        Vector_X = 0,
        Vector_Y = 1,
        Vector_Z = 2
    };
    enum quaternion_component{
        Quat_w = 0,
        Quat_x = 1,
        Quat_y = 2,
        Quat_z = 3
    };

    enum euler_component{
        EULER_ROLL = 0,
        EULER_PITCH,
        EULER_YAW
    };

    Eigen::Vector3f GetGravitationalAcc(){
        Eigen::Vector3f g_I;
        g_I(Vector_X) = 0.0;
        g_I(Vector_Y) = 0.0;
        g_I(Vector_Z) = -9.810;
        return g_I;
    };
}

// 四元数转欧拉角
Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond &q)
{
        // YPR - ZYX
        return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
        // YPR - ZYX
        return Eigen::Quaterniond(
                        Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                        );
}

// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

Eigen::Vector3d quaternion_to_euler2(const Eigen::Vector4f& quat)
{
    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

//rotation matrix to euler anlge
void rotation_to_euler(const Eigen::Matrix3d& dcm, Eigen::Vector3d& euler_angle)
{
    double phi_val = atan2(dcm(2, 1), dcm(2, 2));
    double theta_val = asin(-dcm(2, 0));
    double psi_val = atan2(dcm(1, 0), dcm(0, 0));
    double pi = M_PI;

    if (fabs(theta_val - pi / 2.0) < 1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(dcm(1, 2), dcm(0, 2));

    } else if (fabs(theta_val + pi / 2.0) <  1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(-dcm(1, 2), -dcm(0, 2));
    }

    euler_angle(0) = phi_val;
    euler_angle(1) = theta_val;
    euler_angle(2) = psi_val;
}

//constrain_function
float constrain_function(float data, float Max)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}


//constrain_function2
float constrain_function2(float data, float Min,float Max)
{
    if(data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }else
    {
        return data;
    }
}

// vector based constraint 

Eigen::VectorXf constrain_vector(const Eigen::VectorXf& input, float Max_norm)
{
    Eigen::VectorXf result;

    float norm = input.norm();

    if (norm > Max_norm)
    {
        result = input / norm *  Max_norm;
        return result;
    } else {
        return input;
    }
}

//sign_function
float sign_function(float data)
{
    if(data>0)
    {
        return 1.0;
    }
    else if(data<0)
    {
        return -1.0;
    }
    else if(data == 0)
    {
        return 0.0;
    }
}

// min function
float min(float data1,float data2) {
    if(data1>=data2)
    {
        return data2;
    }
    else
    {
        return data1;
    }
}

Eigen::Vector3f Veemap(const Eigen::Matrix3f& cross_matrix) {
    Eigen::Vector3f vector;
    vector(0) = -cross_matrix(1,2);
    vector(1) = cross_matrix(0,2);
    vector(2) = -cross_matrix(0,1);
    return vector;
}
Eigen::Matrix3f Hatmap(const Eigen::Vector3f& vector) {
    /*

    r^x = [0 -r3 r2;
           r3 0 -r1;
          -r2 r1 0]
    */
    Eigen::Matrix3f cross_matrix;
    cross_matrix(0,0) = 0.0;
    cross_matrix(0,1) = - vector(2);
    cross_matrix(0,2) = vector(1);

    cross_matrix(1,0) = vector(2);
    cross_matrix(1,1) = 0.0;
    cross_matrix(1,2) = - vector(0);

    cross_matrix(2,0) = - vector(1);
    cross_matrix(2,1) = vector(0);
    cross_matrix(2,2) = 0.0;
    return cross_matrix;
}
Eigen::Matrix3f QuaterionToRotationMatrix(const Eigen::Vector4f& quaternion) {
    Eigen::Matrix3f R_IB;

    /* take a special note at the order of the quaterion
    pose[1].q0 = OptiTrackdata.pose.orientation.w;
    pose[1].q1 = OptiTrackdata.pose.orientation.x;
    pose[1].q2 = OptiTrackdata.pose.orientation.y;
    pose[1].q3 = OptiTrackdata.pose.orientation.z;
    
    //update the auxiliary matrix
    /*
    L = [-q1 q0 q3 -q2;
         -q2 -q3 q0 q1;
         -q3 q2 -q1 q0]
    R = [-q1 q0 -q3 q2;
         -q2 q3 q0 -q1;
         -q3 -q2 q1 q0]
    R_IB = RL^T
    */
    Eigen::Matrix<float,3,4> L,R;
    L(0,0) = - quaternion(1);
    L(1,0) = - quaternion(2);
    L(2,0) = - quaternion(3);

    L(0,1) = quaternion(0);
    L(1,2) = quaternion(0);
    L(2,3) = quaternion(0);

    L(0,2) = quaternion(3);
    L(0,3) = - quaternion(2);
    L(1,1) = - quaternion(3);
    L(1,3) = quaternion(1);
    L(2,1) = quaternion(2);
    L(2,2) = - quaternion(1);

    R(0,0) = - quaternion(1);
    R(1,0) = - quaternion(2);
    R(2,0) = - quaternion(3);

    R(0,1) = quaternion(0);
    R(1,2) = quaternion(0);
    R(2,3) = quaternion(0);

    R(0,2) = - quaternion(3);
    R(0,3) =  quaternion(2);
    R(1,1) =  quaternion(3);
    R(1,3) = -quaternion(1);
    R(2,1) = -quaternion(2);
    R(2,2) =  quaternion(1); 

    R_IB = R * L.transpose();
    return R_IB;
}

#endif
