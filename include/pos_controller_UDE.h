/***************************************************************************************************************************
* pos_controller_UDE.h
*
* Author: Qyp
*
* Update Time: 2019.4.12
*
* Introduction:  Position Controller using UDE method
*         1. Ref to Zhongqingchang's paper:
*     Uncertainty and Disturbance Estimator-Based Robust Trajectory Tracking Control for a Quadrotor in a Global Positioning System-Denied Environment
*         2. Ref to : https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
*         3. thrustToAttitude ref to https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
*         4. 没有考虑积分器清零的情况，在降落时 或者突然换方向机动时，积分器需要清0
*         5. 推力到欧拉角基本与PX4吻合，但是在极端情况下不吻合。如：z轴期望值为-100时。
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_UDE_H
#define POS_CONTROLLER_UDE_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <math_utils.h>
#include <px4_command/data_log.h>

#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>


using namespace std;


class pos_controller_UDE
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_UDE(void):
            pos_UDE_nh("~")
        {
            pos_UDE_nh.param<float>("Quad/mass", Quad_MASS, 1.0);
            pos_UDE_nh.param<float>("Quad/throttle_a", throttle_a, 20.0);
            pos_UDE_nh.param<float>("Quad/throttle_b", throttle_b, 0.0);

            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp[0], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp[1], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_z", Kp[2], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd[0], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd[1], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_z", Kd[2], 2.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude[0], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude[1], 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_z", T_ude[2], 1.0);

            pos_UDE_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_UDE_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_UDE_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_UDE_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_UDE_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_UDE_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_UDE_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_UDE_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_UDE_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);
            pos_UDE_nh.param<float>("Limit/THR_MIN", THR_MIN, 0.1);
            pos_UDE_nh.param<float>("Limit/THR_MAX", THR_MAX, 0.9);
            pos_UDE_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_UDE_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);  

            u_l      = Eigen::Vector3f(0.0,0.0,0.0);
            u_d      = Eigen::Vector3f(0.0,0.0,0.0);
            integral = Eigen::Vector3f(0.0,0.0,0.0);
            thrust_sp = Eigen::Vector3d(0.0,0.0,0.0);
        }

        //Quadrotor Parameter
        float Quad_MASS;
        float throttle_a;
        float throttle_b;

        //UDE control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float THR_MIN;
        float THR_MAX;
        float tilt_max;
        float int_start_error;

        //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector3f u_l,u_d;

        Eigen::Vector3f integral;

        Eigen::Vector3d thrust_sp;

        //Printf the UDE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        Eigen::Vector3d pos_controller(px4_command::DroneState _DroneState, px4_command::TrajectoryPoint _Reference_State, float dt);

    private:

        ros::NodeHandle pos_UDE_nh;

};

Eigen::Vector3d pos_controller_UDE::pos_controller(
    px4_command::DroneState _DroneState, 
    px4_command::TrajectoryPoint _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;
    Eigen::Vector3d thrust_sp;

    // 计算误差项
    Eigen::Vector3f pos_error = px4_command_utils::cal_pos_error(_DroneState, _Reference_State);
    Eigen::Vector3f vel_error = px4_command_utils::cal_vel_error(_DroneState, _Reference_State);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }

    // UDE算法
    for (int i = 0; i < 3; i++)
    {
        u_l[i] = _Reference_State.acceleration_ref[i] + Kp[i] * pos_error[i] + Kd[i] * vel_error[i];
        u_d[i] = - 1.0 / T_ude[i] * (Kd[i] * pos_error[i] + vel_error[i] + Kp[i] * integral[i]);
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * dt;
        }else
        {
            integral[i] = 0;
        }

        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }

        if(abs(u_d[i]) > int_max[i])
        {
            cout << "u_d saturation! " << " [0-1-2] "<< i <<endl;
            cout << "[u_d]: "<< u_d[i]<<" [u_d_max]: "<<int_max[i]<<" [m/s] "<<endl;
        }

        u_d[i] = constrain_function(u_d[i], int_max[i]);
    }

    // 期望加速度
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.8;

    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    for (int i=0; i<3; i++)
    {
        thrust_sp[i] = (accel_sp[i] * Quad_MASS - throttle_b) / throttle_a;
    }

    // 推力限幅，根据最大倾斜角及最大油门
    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    float thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(THR_MAX * THR_MAX - pow(thrust_sp[2],2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    if ((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)) > thrust_max_XY * thrust_max_XY) {
        float mag = sqrtf((pow(thrust_sp[0],2) + pow(thrust_sp[1],2)));
        thrust_sp[0] = thrust_sp[0] / mag * thrust_max_XY;
        thrust_sp[1] = thrust_sp[1] / mag * thrust_max_XY;
    }

    return thrust_sp;
}

void pos_controller_UDE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>PD+UDE Position Controller<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
    
    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    cout << "int [X Y Z] : " << integral[0] << " [N] "<< integral[1]<<" [N] "<<integral[2]<<" [N] "<<endl;
    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;

    cout << "thrust_sp    [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;

}

// 【打印参数函数】
void pos_controller_UDE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>UDE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout <<"Quad_MASS : "<< Quad_MASS << endl;
    cout <<"throttle_a : "<< throttle_a << endl;
    cout <<"throttle_b : "<< throttle_b << endl;

    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"Kd_x : "<< Kd[0] << endl;
    cout <<"Kd_y : "<< Kd[1] << endl;
    cout <<"Kd_z : "<< Kd[2] << endl;

    cout <<"T_ude_x : "<< T_ude[0] << endl;
    cout <<"T_ude_y : "<< T_ude[1] << endl;
    cout <<"T_ude_z : "<< T_ude[2] << endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"THR_MIN : "<< THR_MIN << endl;
    cout <<"THR_MAX : "<< THR_MAX << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

}


#endif
