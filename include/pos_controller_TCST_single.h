#ifndef POS_CONTROLLER_TIE_H
#define POS_CONTROLLER_TIE_H

/*

Author: Longhao Qian
Date : 2019 12 16

Robust control from TCST paper for single UAV payload

Visual feedback from April Tag

*/

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <math_utils.h>

#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/ControlOutput.h>

using namespace std;

class pos_controller_TCST_single
{
    public:

        //构造函数
        pos_controller_TCST_single(void):
            pos_tcst_single_nh("~")
        {
            pos_tcst_single_nh.param<float>("Quad/mass", Quad_MASS, 1.0);
            pos_tcst_singlenh.param<float>("Payload/mass", Payload_Mass, 1.0);
            pos_tcst_single_nh.param<float>("Cable/length", Cable_Length, 1.0);
            pos_tcst_single_nh.param<float>("Pos_tcst_single/T_tcst_single_xy", T_tcst_single[0], 1.0);
            pos_tcst_single_nh.param<float>("Pos_tcst_single/T_tcst_single_xy", T_tcst_single[1], 1.0);
            pos_tcst_single_nh.param<float>("Pos_tcst_single/T_tcst_single_z",  T_tcst_single[2], 1.0);

            pos_tcst_single_nh.param<float>("Pos_tie/Integration", integration, 1.0);

            pos_tcst_single_nh.param<float>("Pos_tie/Kp_xy", Kp[0], 1.0);
            pos_tcst_singlenh.param<float>("Pos_tie/Kp_xy", Kp[1], 1.0);
            pos_tcst_single_nh.param<float>("Pos_tie/Kp_z" , Kp[2], 2.0);
            pos_tcst_single_nh.param<float>("Pos_tie/Kv_xy", Kv[0], 0.5);
            pos_tcst_single_nh.param<float>("Pos_tie/Kv_xy", Kv[1], 0.5);
            pos_tcst_single_nh.param<float>("Pos_tie/Kv_z",  Kv[2], 0.5);
            pos_tcst_single_nh.param<float>("Pos_tie/Kpv_xy", Kpv[0], 0.0);
            pos_tcst_single_nh.param<float>("Pos_tie/Kpv_xy", Kpv[1], 0.0);
            pos_tcst_single_nh.param<float>("Pos_tie/Kpv_z",  Kpv[2], 0.0);
            pos_tcst_single_nh.param<float>("Pos_tie/KL", KL, 0.0);

            pos_tcst_single_nh.param<float>("Limitne/pxy_error_max", pos_error_max[0], 0.6);
            pos_tcst_single_nh.param<float>("Limitne/pxy_error_max", pos_error_max[1], 0.6);
            pos_tcst_single_nh.param<float>("Limit/pz_error_max" ,   pos_error_max[2], 1.0);
            pos_tcst_single_nh.param<float>("Limit/vxy_error_max",   vel_error_max[0], 0.3);
            pos_tcst_single_nh.param<float>("Limit/vxy_error_max",  vel_error_max[1], 0.3);
            pos_tcst_single_nh.param<float>("Limit/vz_error_max" ,  vel_error_max[2], 1.0);
            pos_tcst_single_nh.param<float>("Limit/pxy_int_max"  ,  int_max[0], 0.5);
            pos_tcst_single_nh.param<float>("Limit/pxy_int_max"  ,  int_max[1], 0.5);
            pos_tcst_single_nh.param<float>("Limit/pz_int_max"   ,  int_max[2], 0.5);
            pos_tcst_single_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_tcst_single_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            u_l      = Eigen::Vector3f(0.0,0.0,0.0);
            u_d      = Eigen::Vector3f(0.0,0.0,0.0);
            u_p      = Eigen::Vector3f(0.0,0.0,0.0);
            r        = Eigen::Vector2f(0.0,0.0);
            v_p      = Eigen::Vector2f(0.0,0.0);
            integral = Eigen::Vector3f(0.0,0.0,0.0);
            B<<1.0,0.0,
               0.0,1.0,
               0.0,0.0;
            Cable_Length_sq = Cable_Length*Cable_Length;
        }
        float integration;
        //Quadrotor and Payload Parameter
        float Quad_MASS;
        float Payload_Mass;
        float Cable_Length;
        float Cable_Length_sq;
        //Controller parameter for the control law
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kv;
        Eigen::Vector3f T_tie;
        Eigen::Vector3f Kpv;
        float KL;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float tilt_max;
        float int_start_error;

        //积分项
        Eigen::Vector3f u_l,u_d,u_p; //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector2f v_p, r;
        Eigen::Matrix<float, 3,2> B;
        Eigen::Vector3f integral;

        px4_command::ControlOutput _ControlOutput;


        //Printf the TIE parameter
        void printf_param();

        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        px4_command::ControlOutput pos_controller(const px4_command::DroneState& _DroneState, const px4_command::TrajectoryPoint& _Reference_State, float dt);

    private:
        ros::NodeHandle pos_tie_nh;

};

px4_command::ControlOutput pos_controller_TIE::pos_controller(
    const px4_command::DroneState& _DroneState, 
    const px4_command::TrajectoryPoint& _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;
    
    // 计算误差项
    Eigen::Vector3f pos_error;
    Eigen::Vector3f vel_error;
    
    px4_command_utils::cal_pos_error(_DroneState, _Reference_State, pos_error);
    px4_command_utils::cal_vel_error(_DroneState, _Reference_State, vel_error);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }

// control law form the tie paper
    float payload_relative_vel[3];
    float payload_relative_pos[3];

    for(int i=0;i<3;i++)
    {
        payload_relative_vel[i] = _DroneState.payload_vel[i] -  _DroneState.velocity[i];// get payload relative velocity
        payload_relative_pos[i] = _DroneState.payload_pos[i] -  _DroneState.position[i];// get payload relative velocity
    }

    for (int i = 0;i<2;i++)
    {
        r(i) = payload_relative_pos[i];
        v_p(i) = payload_relative_vel[i];
    }
    
    float sq_r = r(0)*r(0) + r(1)*r(1);

    if (Cable_Length_sq - sq_r>0.01)
    {
        B(2,0) =  r(0)/sqrt((Cable_Length_sq - sq_r));
        B(2,1) =  r(1)/sqrt((Cable_Length_sq - sq_r));
    }else{
        B(2,0) = 0.1;
        B(2,1) = 0.1;
    }

    u_p = B*(v_p + KL*r);
    for(int i = 0;i<3;i++)
    {
        u_p[i] = Kpv[i] * u_p[i];
    }


    for (int i = 0; i < 3; i++)
    {

        u_l[i] = Kp[i] * pos_error[i] + Kv[i] * vel_error[i] + u_p[i];
        // additional feedback based on payload relative position:
        u_d[i] = 1.0* integration / T_tie[i] * (Payload_Mass*(1+Kpv[i])* payload_relative_vel[i] + Quad_MASS*_DroneState.velocity[i]  + integral[i])/(Payload_Mass+Quad_MASS);
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) < int_start_error)
        {
            //integral[i] += pos_error[i] * dt;
            integral[i] += ( -Kp[i]*pos_error[i] - Kv[i] * vel_error[i]) * dt;

            if(abs(integral[i]) > int_max[i])
            {
                cout << "Integral saturation! " << " [0-1-2] "<< i <<endl;
                cout << "[integral]: "<< integral[i]<<" [int_max]: "<<int_max[i]<<" [m/s] "<<endl;
            }

            integral[i] = constrain_function(integral[i], int_max[i]);
        }else
        {
            integral[i] = 0;
        }

        // If not in OFFBOARD mode, set all intergral to zero.
        if(_DroneState.mode != "OFFBOARD")
        {
            integral[i] = 0;
        }
        u_d[i] = constrain_function(u_d[i], int_max[i]);
    }
    // 期望加速度
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.8;
    
    // 期望推力 = 期望加速度 × 质量
    // 归一化推力 ： 根据电机模型，反解出归一化推力
    Eigen::Vector3d thrust_sp;
    Eigen::Vector3d throttle_sp;
    thrust_sp =  px4_command_utils::accelToThrust(accel_sp, Payload_Mass+Quad_MASS, tilt_max);
    throttle_sp = px4_command_utils::thrustToThrottle(thrust_sp);

    for (int i=0; i<3; i++)
    {
        _ControlOutput.u_l[i] = u_l[i];
        _ControlOutput.u_d[i] = u_d[i];
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;

}

void pos_controller_TIE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  tie Position Controller  <<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
}

// 【打印参数函数】
void pos_controller_TIE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Payload control method in TIE paper (Parameter)  <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;
    cout <<"Payload_MASS : "<< Payload_Mass << endl;
    cout <<"Cable_Length : "<< Cable_Length << endl;

    cout <<"Kp_x : "<< Kp[0] << endl;
    cout <<"Kp_y : "<< Kp[1] << endl;
    cout <<"Kp_z : "<< Kp[2] << endl;

    cout <<"T_tie_x : "<< T_tie[0] << endl;
    cout <<"T_tie_y : "<< T_tie[1] << endl;
    cout <<"T_tie_z : "<< T_tie[2] << endl;

    cout <<"Kv_x : "<< Kv[0] << endl;
    cout <<"Kv_y : "<< Kv[1] << endl;
    cout <<"Kv_z : "<< Kv[2] << endl;

    cout <<"Kpv_x"<< Kpv[0]<<endl;
    cout <<"Kpv_y"<< Kpv[1]<<endl;
    cout <<"Kpv_z"<< Kpv[2]<<endl;

    cout <<"Limit:  " <<endl;
    cout <<"pxy_error_max : "<< pos_error_max[0] << endl;
    cout <<"pz_error_max :  "<< pos_error_max[2] << endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"pxy_int_max : "<< int_max[0] << endl;
    cout <<"pz_int_max : "<< int_max[2] << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;

}

#endif
