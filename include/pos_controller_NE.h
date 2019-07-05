/***************************************************************************************************************************
* pos_controller_NE.h
*
* Author: Qyp
*
* Update Time: 2019.5.1
*
* Introduction:  Position Controller using NE+UDE method
***************************************************************************************************************************/
#ifndef POS_CONTROLLER_NE_H
#define POS_CONTROLLER_NE_H

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <pos_controller_utils.h>
#include <math_utils.h>

#include <LowPassFilter.h>
#include <HighPassFilter.h>
#include <LeadLagFilter.h>
#include <px4_command/data_log.h>

#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>


using namespace std;
 
class pos_controller_NE
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_NE(void):
            pos_NE_nh("~")
        {
            pos_NE_nh.param<float>("Quad/mass", Quad_MASS, 1.0);
            pos_NE_nh.param<float>("Quad/throttle_a", throttle_a, 20.0);
            pos_NE_nh.param<float>("Quad/throttle_b", throttle_b, 0.0);

            pos_NE_nh.param<float>("Pos_ne/Kp_xy", Kp[0], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kp_xy", Kp[1], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kp_z",  Kp[2], 1.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_xy", Kd[0], 2.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_xy", Kd[1], 2.0);
            pos_NE_nh.param<float>("Pos_ne/Kd_z",  Kd[2], 2.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_xy", T_ude[0], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_xy", T_ude[1], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ude_z", T_ude[2], 1.0);
            pos_NE_nh.param<float>("Pos_ne/T_ne", T_ne, 1.0);

            pos_NE_nh.param<float>("Limit/pxy_error_max", pos_error_max[0], 0.6);
            pos_NE_nh.param<float>("Limit/pxy_error_max", pos_error_max[1], 0.6);
            pos_NE_nh.param<float>("Limit/pz_error_max" , pos_error_max[2], 1.0);
            pos_NE_nh.param<float>("Limit/vxy_error_max", vel_error_max[0], 0.3);
            pos_NE_nh.param<float>("Limit/vxy_error_max", vel_error_max[1], 0.3);
            pos_NE_nh.param<float>("Limit/vz_error_max" , vel_error_max[2], 1.0);
            pos_NE_nh.param<float>("Limit/pxy_int_max"  , int_max[0], 0.5);
            pos_NE_nh.param<float>("Limit/pxy_int_max"  , int_max[1], 0.5);
            pos_NE_nh.param<float>("Limit/pz_int_max"   , int_max[2], 0.5);  
            pos_NE_nh.param<float>("Limit/THR_MIN", THR_MIN, 0.1);
            pos_NE_nh.param<float>("Limit/THR_MAX", THR_MAX, 0.9);
            pos_NE_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);
            pos_NE_nh.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            u_l             = Eigen::Vector3f(0.0,0.0,0.0);
            u_d             = Eigen::Vector3f(0.0,0.0,0.0);
            integral        = Eigen::Vector3f(0.0,0.0,0.0);
            integral_LLF    = Eigen::Vector3f(0.0,0.0,0.0);
            NoiseEstimator  = Eigen::Vector3f(0.0,0.0,0.0);
            output_LLF      = Eigen::Vector3f(0.0,0.0,0.0);
            thrust_sp = Eigen::Vector3d(0.0,0.0,0.0);

            set_filter();
        }


        //Quadrotor Parameter
        float Quad_MASS;
        float throttle_a;
        float throttle_b;

        //Limitation
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        Eigen::Vector3f int_max;
        float THR_MIN;
        float THR_MAX;
        float tilt_max;
        float int_start_error;

        //NE control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;
        float T_ne;

        //Filter for NE
        LowPassFilter LPF_x;
        LowPassFilter LPF_y;
        LowPassFilter LPF_z;

        HighPassFilter HPF_x;
        HighPassFilter HPF_y;
        HighPassFilter HPF_z;

        LeadLagFilter LLF_x;
        LeadLagFilter LLF_y;
        LeadLagFilter LLF_z;

        //u_l for nominal contorol(PD), u_d for NE control(disturbance estimator)
        Eigen::Vector3f u_l,u_d;

        Eigen::Vector3f integral;

        Eigen::Vector3f integral_LLF;

        Eigen::Vector3f output_LLF;

        Eigen::Vector3d pos_initial;

        Eigen::Vector3f NoiseEstimator;

        Eigen::Vector3d thrust_sp;

        //Printf the NE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        // Position control main function 
        // [Input: Current state, Reference state, sub_mode, dt; Output: AttitudeReference;]
        Eigen::Vector3d pos_controller(px4_command::DroneState _DroneState, px4_command::TrajectoryPoint _Reference_State, float dt);

        Eigen::Vector3f set_initial_pos(Eigen::Vector3d pos);

        void set_filter();

    private:

        ros::NodeHandle pos_NE_nh;

};

void pos_controller_NE::set_filter()
{
    LPF_x.set_Time_constant(T_ne);
    LPF_y.set_Time_constant(T_ne);
    LPF_z.set_Time_constant(T_ne);

    HPF_x.set_Time_constant(T_ne);
    HPF_y.set_Time_constant(T_ne);
    HPF_z.set_Time_constant(T_ne);

    LLF_x.set_Time_constant(T_ne, Kd[0]);
    LLF_y.set_Time_constant(T_ne, Kd[1]);
    LLF_z.set_Time_constant(T_ne, Kd[2]);
}

Eigen::Vector3f pos_controller_NE::set_initial_pos(Eigen::Vector3d pos)
{
    pos_initial = pos;
}

Eigen::Vector3d pos_controller_NE::pos_controller(
    px4_command::DroneState _DroneState, 
    px4_command::TrajectoryPoint _Reference_State, float dt)
{
    Eigen::Vector3d accel_sp;
    Eigen::Vector3d thrust_sp;
    // 计算误差项
    Eigen::Vector3f pos_error = pos_controller_utils::cal_pos_error(_DroneState, _Reference_State);
    Eigen::Vector3f vel_error = pos_controller_utils::cal_vel_error(_DroneState, _Reference_State);

    // 误差项限幅
    for (int i=0; i<3; i++)
    {
        pos_error[i] = constrain_function(pos_error[i], pos_error_max[i]);
        vel_error[i] = constrain_function(vel_error[i], vel_error_max[i]);
    }

    //Noise estimator
    NoiseEstimator[0] = LPF_x.apply(_DroneState.velocity[0], dt) + HPF_x.apply(pos_initial[0] - _DroneState.position[0], dt);
    NoiseEstimator[1] = LPF_y.apply(_DroneState.velocity[1], dt) + HPF_y.apply(pos_initial[1] - _DroneState.position[1], dt);
    NoiseEstimator[2] = LPF_z.apply(_DroneState.velocity[2], dt) + HPF_z.apply(pos_initial[2] - _DroneState.position[2], dt);

    //u_l
    for (int i = 0; i < 3; i++)
    {
       u_l[i] = _Reference_State.acceleration_ref[i] +  Kp[i] * pos_error[i] + Kd[i] * ( vel_error[i] + NoiseEstimator[i]);
    }

    //UDE term
    Eigen::Vector3f input_LLF;

    for (int i = 0; i < 3; i++)
    {
        integral_LLF[i] = integral_LLF[i] +  _DroneState.velocity[i] * dt;
        input_LLF[i] =  integral_LLF[i] - _DroneState.position[i] + pos_initial[i];
    }

    output_LLF[0] = LLF_x.apply(input_LLF[0], dt);
    output_LLF[1] = LLF_y.apply(input_LLF[1], dt);
    output_LLF[2] = LLF_z.apply(input_LLF[2], dt);

    for (int i = 0; i < 3; i++)
    {
        u_d[i] = 1.0 / T_ude[i] * ( _DroneState.velocity[i] - output_LLF[i] - integral[i] );
    }

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        integral[i] = integral[i] +  ( _Reference_State.acceleration_ref[i] +  Kp[i] * pos_error[i] + Kd[i] * vel_error[i]) * dt;

        // If not in OFFBOARD mode, set all intergral to zero.
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

    //     cout <<">>>>>>>>>>>>>>>>>>>>>>PID Position Controller<<<<<<<<<<<<<<<<<<<<<" <<endl;

    // //固定的浮点显示
    // cout.setf(ios::fixed);
    // //左对齐
    // cout.setf(ios::left);
    // // 强制显示小数点
    // cout.setf(ios::showpoint);
    // // 强制显示符号
    // cout.setf(ios::showpos);

    // cout<<setprecision(2);

    // cout << "e_p [X Y Z] : " << pos_error[0] << " [m] "<< pos_error[1]<<" [m] "<<pos_error[2]<<" [m] "<<endl;
    // cout << "e_v [X Y Z] : " << vel_error[0] << " [m/s] "<< vel_error[1]<<" [m/s] "<<vel_error[2]<<" [m/s] "<<endl;
    // cout << "acc_ref [X Y Z] : " << _Reference_State.acceleration_ref[0] << " [m/s^2] "<< _Reference_State.acceleration_ref[1]<<" [m/s^2] "<<_Reference_State.acceleration_ref[2]<<" [m/s^2] "<<endl;
    
    // cout << "desired_acceleration [X Y Z] : " << _AttitudeReference.desired_acceleration[0] << " [m/s^2] "<< _AttitudeReference.desired_acceleration[1]<<" [Nm/s^2] "<<_AttitudeReference.desired_acceleration[2]<<" [m/s^2] "<<endl;
    // cout << "desired_thrust [X Y Z] : " << _AttitudeReference.desired_thrust[0] << " [N] "<< _AttitudeReference.desired_thrust[1]<<" [N] "<<_AttitudeReference.desired_thrust[2]<<" [N] "<<endl;
    // cout << "desired_thrust_normalized [X Y Z] : " << _AttitudeReference.desired_thrust_normalized[0] << " [N] "<< _AttitudeReference.desired_thrust_normalized[1]<<" [N] "<<_AttitudeReference.desired_thrust_normalized[2]<<" [N] "<<endl;
    // cout << "desired_attitude [R P Y] : " << _AttitudeReference.desired_attitude[0] * 180/M_PI <<" [deg] "<<_AttitudeReference.desired_attitude[1] * 180/M_PI << " [deg] "<< _AttitudeReference.desired_attitude[2] * 180/M_PI<<" [deg] "<<endl;
    // cout << "desired_throttle [0-1] : " << _AttitudeReference.desired_throttle <<endl;

   // return _AttitudeReference;
}

void pos_controller_NE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>NE Position Controller<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);

    // cout << "NoiseEstimator [X Y Z] : " <<  Quad_MASS *Kd[0] * NoiseEstimator[0] << " [N] "<<Quad_MASS *Kd[1] * NoiseEstimator[1]<<" [N] "<<Quad_MASS *Kd[2] *NoiseEstimator[2]<<" [N] "<<endl;

    // cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    // cout << "output_LLF [X Y Z] : " << output_LLF[0] << " [N] "<< output_LLF[1]<<" [N] "<<output_LLF[2]<<" [N] "<<endl;

    // cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
    cout << "thrust_sp    [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;
}

// 【打印参数函数】
void pos_controller_NE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>NE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;
    cout <<"throttle_a : "<< throttle_a << endl;
    cout <<"throttle_b : "<< throttle_b << endl;

    cout <<"Kp_X : "<< Kp[0] << endl;
    cout <<"Kp_Y : "<< Kp[1] << endl;
    cout <<"Kp_Z : "<< Kp[2] << endl;

    cout <<"Kd_X : "<< Kd[0] << endl;
    cout <<"Kd_Y : "<< Kd[1] << endl;
    cout <<"Kd_Z : "<< Kd[2] << endl;

    cout <<"NE_T_X : "<< T_ude[0] << endl;
    cout <<"NE_T_Y : "<< T_ude[1] << endl;
    cout <<"NE_T_Z : "<< T_ude[2] << endl;
    cout <<"T_ne : "<< T_ne << endl;

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

    cout <<"Filter_LPFx : "<< LPF_x.get_Time_constant()<<" Filter_LPFy : "<< LPF_y.get_Time_constant()<<" Filter_LPFz : "<< LPF_z.get_Time_constant() << endl;
    cout <<"Filter_HPFx : "<< HPF_x.get_Time_constant()<<" Filter_HPFy : "<< HPF_y.get_Time_constant()<<" Filter_HPFz : "<< HPF_z.get_Time_constant() << endl;
    cout <<"Filter_LLFx : "<< LLF_x.get_Time_constant()<<" Filter_LLFy : "<< LLF_y.get_Time_constant()<<" Filter_LLFz : "<< LLF_z.get_Time_constant() << endl;

    cout <<"kd_LLFx : "<< LLF_x.get_Kd() <<" kd_LLFy : "<< LLF_y.get_Kd() <<" kd_LLFz : "<< LLF_z.get_Kd() << endl;


}


#endif
