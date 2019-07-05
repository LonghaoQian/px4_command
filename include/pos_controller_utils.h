/***************************************************************************************************************************
* pos_controller_utils.h
*
* Author: Qyp
*
* Update Time: 2019.6.28
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_UTILS_H
#define POS_CONTROLLER_UTILS_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>

using namespace std;

namespace pos_controller_utils {

//计算位置误差
Eigen::Vector3f cal_pos_error(px4_command::DroneState _DroneState, px4_command::TrajectoryPoint _Reference_State)
{
    Eigen::Vector3f pos_error;

    for (int i=0; i<3; i++)
    {
        pos_error[i] = _Reference_State.position_ref[i] - _DroneState.position[i];
    }

    // 对于速度追踪子模式，则无位置反馈
    if(_Reference_State.Sub_mode == command_to_mavros::XY_VEL_Z_POS || _Reference_State.Sub_mode == command_to_mavros::XY_VEL_Z_VEL) 
    {
        for (int i=0; i<2; i++)
        {
            pos_error[i] = 0;
        }
    }

    if(_Reference_State.Sub_mode == command_to_mavros::XY_POS_Z_VEL || _Reference_State.Sub_mode == command_to_mavros::XY_VEL_Z_VEL) 
    {
        pos_error[3] = 0;
    }

    return pos_error;
}

//计算速度误差
Eigen::Vector3f cal_vel_error(px4_command::DroneState _DroneState, px4_command::TrajectoryPoint _Reference_State)
{
    Eigen::Vector3f vel_error;

    for (int i=0; i<3; i++)
    {
        vel_error[i] = _Reference_State.velocity_ref[i] - _DroneState.velocity[i];
    }

    return vel_error;
}


//Throttle to Attitude
//Thrust to Attitude
//Input: desired thrust (desired throttle [0,1]) and yaw_sp(rad)
//Output: desired attitude (quaternion)

px4_command::AttitudeReference thrustToAttitude(Eigen::Vector3d thr_sp, float yaw_sp)
{
    px4_command::AttitudeReference _AttitudeReference;

    Eigen::Vector3d att_sp;
    att_sp[2] = yaw_sp;

    // desired body_z axis = -normalize(thrust_vector)
    Eigen::Vector3d body_x, body_y, body_z;

    double thr_sp_length = thr_sp.norm();

    //cout << "thr_sp_length : "<< thr_sp_length << endl;

    if (thr_sp_length > 0.00001f) {
            body_z = thr_sp.normalized();

    } else {
            // no thrust, set Z axis to safe value
            body_z = Eigen::Vector3d(0.0f, 0.0f, 1.0f);
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    Eigen::Vector3d y_C = Eigen::Vector3d(-sinf(yaw_sp),cosf(yaw_sp),0.0);

    if (fabsf(body_z(2)) > 0.000001f) {
            // desired body_x axis, orthogonal to body_z
            body_x = y_C.cross(body_z);

            // keep nose to front while inverted upside down
            if (body_z(2) < 0.0f) {
                    body_x = -body_x;
            }

            body_x.normalize();

    } else {
            // desired thrust is in XY plane, set X downside to construct correct matrix,
            // but yaw component will not be used actually
            body_x = Eigen::Vector3d(0.0f, 0.0f, 0.0f);
            body_x(2) = 1.0f;
    }

    // desired body_y axis
    body_y = body_z.cross(body_x);

    Eigen::Matrix3d R_sp;

    // fill rotation matrix
    for (int i = 0; i < 3; i++) {
            R_sp(i, 0) = body_x(i);
            R_sp(i, 1) = body_y(i);
            R_sp(i, 2) = body_z(i);
    }

    Eigen::Quaterniond q_sp(R_sp);

    att_sp = rotation_to_euler(R_sp);

    //cout << "Desired euler [R P Y]: "<< att_sp[0]* 180/M_PI <<" [deg] " << att_sp[1]* 180/M_PI <<" [deg] "<< att_sp[2]* 180/M_PI <<" [deg] "<< endl;
    //cout << "Desired Thrust: "<< thr_sp_length<< endl;
//    cout << "q_sp [x y z w]: "<< q_sp.x() <<" [ ] " << q_sp.y() <<" [ ] "<<q_sp.z() <<" [ ] "<<q_sp.w() <<" [ ] "<<endl;
//    cout << "R_sp : "<< R_sp(0, 0) <<" " << R_sp(0, 1) <<" "<< R_sp(0, 2) << endl;
//    cout << "     : "<< R_sp(1, 0) <<" " << R_sp(1, 1) <<" "<< R_sp(1, 2) << endl;
//    cout << "     : "<< R_sp(2, 0) <<" " << R_sp(2, 1) <<" "<< R_sp(2, 2) << endl;

    _AttitudeReference.desired_att_q.w = q_sp.w();
    _AttitudeReference.desired_att_q.x = q_sp.x();
    _AttitudeReference.desired_att_q.y = q_sp.y();
    _AttitudeReference.desired_att_q.z = q_sp.z();

    _AttitudeReference.desired_attitude[0] = att_sp[0];  
    _AttitudeReference.desired_attitude[1] = att_sp[1]; 
    _AttitudeReference.desired_attitude[2] = att_sp[2]; 

    //期望油门
    _AttitudeReference.desired_throttle = thr_sp_length;  

    return _AttitudeReference;
}

}
#endif
