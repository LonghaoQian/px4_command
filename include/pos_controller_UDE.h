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
#include <math_utils.h>
#include <px4_command/data_log.h>

using namespace std;

namespace namespace_UDE {

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

            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp(0), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_xy", Kp(1), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kp_z", Kp(2), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd(0), 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_xy", Kd(1), 2.0);
            pos_UDE_nh.param<float>("Pos_ude/Kd_z", Kd(2), 2.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude(0), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_xy", T_ude(1), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/T_ude_z", T_ude(2), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/INT_LIM_X", INT_LIM(0), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/INT_LIM_Y", INT_LIM(1), 1.0);
            pos_UDE_nh.param<float>("Pos_ude/INT_LIM_Z", INT_LIM(2), 5.0);

            pos_UDE_nh.param<float>("Limit/XY_VEL_MAX", XY_VEL_MAX, 1.0);
            pos_UDE_nh.param<float>("Limit/Z_VEL_MAX", Z_VEL_MAX, 1.0);
            pos_UDE_nh.param<float>("Limit/THR_MIN", THR_MIN, 0.1);
            pos_UDE_nh.param<float>("Limit/THR_MAX", THR_MAX, 0.9);
            pos_UDE_nh.param<float>("Limit/tilt_max", tilt_max, 20.0);

            thrust_sp       = Eigen::Vector3d(0.0,0.0,0.0);
            u_l             = Eigen::Vector3d(0.0,0.0,0.0);
            u_d             = Eigen::Vector3d(0.0,0.0,0.0);
            u_total         = Eigen::Vector3d(0.0,0.0,0.0);
            integral_ude    = Eigen::Vector3d(0.0,0.0,0.0);
            error_pos       = Eigen::Vector3d(0.0,0.0,0.0);
            error_vel       = Eigen::Vector3d(0.0,0.0,0.0);

            delta_time      = 0.0;
            flag_offboard   = 0;

            state_sub = pos_UDE_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &pos_controller_UDE::state_cb,this);
            data_log_pub = pos_UDE_nh.advertise<px4_command::data_log>("/px4_command/data_log", 10);


        }

        //Mass of the quadrotor
        float Quad_MASS;

        //UDE control parameter
        Eigen::Vector3f Kp;
        Eigen::Vector3f Kd;
        Eigen::Vector3f T_ude;

        //Limitation of UDE integral
        Eigen::Vector3f INT_LIM;

        //Limitation of the velocity
        float XY_VEL_MAX;
        float Z_VEL_MAX;

        //Limitation of the thrust
        float THR_MIN;
        float THR_MAX;

        //Limitation of the tilt angle (roll and pitch)  [degree]
        float tilt_max;

        float throttle_a;
        float throttle_b;

        Eigen::Vector3d error_pos,error_vel;

        //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector3d u_l,u_d,u_total;

        Eigen::Vector3d integral_ude;

        //Desired thurst of the drone[the output of this class]
        Eigen::Vector3d thrust_sp;

        //The delta time between now and the last step
        float delta_time;

        //Current state of the drone
        mavros_msgs::State current_state;

        //for log the control state
        px4_command::data_log data_log;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;

        //Printf the UDE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        //Position control main function [Input: current pos, current vel, desired state(pos or vel), sub_mode, time_now; Output: desired thrust;]
        Eigen::Vector3d pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, float dt);

    private:

        ros::NodeHandle pos_UDE_nh;

        ros::Subscriber state_sub;
        ros::Publisher data_log_pub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            current_state = *msg;

            if(current_state.mode == "OFFBOARD")
            {
                flag_offboard = 1;
            }else
            {
                flag_offboard = 0;
            }

        }

};


Eigen::Vector3d pos_controller_UDE::pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, float dt)
{
    delta_time = dt;

    error_pos = pos_sp - pos;
    error_vel =  - vel;
    for (int i = 0; i < 3; i++)
    {
        error_pos(i) = constrain_function2(error_pos(i), -1.0, 1.0);
        u_l(i) = Quad_MASS * (Kp(i) * error_pos(i) + Kd(i) * error_vel(i));
        u_d(i) = Quad_MASS / T_ude(i) * (Kd(i) * error_pos(i) + error_vel(i) + Kp(i) * integral_ude(i));
    }


    /* explicitly limit the integrator state */
    for (int i = 0; i < 3; i++)
    {
        // Perform the integration using a first order method and do not propagate the result if out of range or invalid

        float integral = 0;
        if(error_pos(i) < 2.0)
        {
            integral = integral_ude(i) +  error_pos(i) * delta_time;
        }

        integral_ude(i) = integral;

        u_d(i) = constrain_function2(u_d(i), -INT_LIM(i), INT_LIM(i));
    }

    //ENU frame
    u_total(0) = u_l(0) + u_d(0);
    u_total(1) = u_l(1) + u_d(1);
    u_total(2) = u_l(2) + u_d(2) + Quad_MASS * 9.8;

    //Thrust to scale thrust[0,1]
    Eigen::Vector3d thrust_sp_scale;
    thrust_sp_scale(0) = (u_total(0) - throttle_b) / throttle_a;
    thrust_sp_scale(1) = (u_total(1) - throttle_b) / throttle_a;
    thrust_sp_scale(2) = (u_total(2) - throttle_b) / throttle_a;

    //Limit the Thrust
    thrust_sp(2) = constrain_function2( thrust_sp_scale(2) , THR_MIN, THR_MAX);

    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    float thrust_max_XY_tilt = fabs(thrust_sp(2)) * tanf(tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(THR_MAX * THR_MAX - thrust_sp(2) * thrust_sp(2));
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);

    // Saturate thrust in XY-direction.
    thrust_sp(0) = thrust_sp_scale(0);
    thrust_sp(1) = thrust_sp_scale(1);

    if ((thrust_sp_scale(0) * thrust_sp_scale(0) + thrust_sp_scale(1) * thrust_sp_scale(1)) > thrust_max_XY * thrust_max_XY) {
            float mag = sqrtf((thrust_sp_scale(0) * thrust_sp_scale(0) + thrust_sp_scale(1) * thrust_sp_scale(1)));
            thrust_sp(0) = thrust_sp_scale(0) / mag * thrust_max_XY;
            thrust_sp(1) = thrust_sp_scale(1) / mag * thrust_max_XY;
    }

    //If not in OFFBOARD mode, set all intergral to zero.
    if(flag_offboard == 0)
    {
        integral_ude = Eigen::Vector3d(0.0,0.0,0.0);
    }

    for (int i = 0; i < 3; i++)
    {
        data_log.pos[i] = pos(i);
        data_log.vel[i] = vel(i);
        data_log.pos_sp[i] = pos_sp(i);
        data_log.u_l[i] = u_l(i);
        data_log.u_d[i] = u_d(i);
        data_log.u_total[i] = u_total(i);
        data_log.thrust_sp[i] = thrust_sp(i);
    }

    data_log_pub.publish(data_log);

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

    cout << "delta_time : " << delta_time<< " [s] " <<endl;

    cout << "e_p [X Y Z] : " << error_pos[0] << " [N] "<< error_pos[1]<<" [N] "<<error_pos[2]<<" [N] "<<endl;
    cout << "e_v [X Y Z] : " << error_vel[0] << " [N] "<< error_vel[1]<<" [N] "<<error_vel[2]<<" [N] "<<endl;
    cout << "int [X Y Z] : " << integral_ude[0] << " [N] "<< integral_ude[1]<<" [N] "<<integral_ude[2]<<" [N] "<<endl;

    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;

    cout << "u_total [X Y Z] : " << u_total[0] << " [N] "<< u_total[1]<<" [N] "<<u_total[2]<<" [N] "<<endl;

    cout << "thrust_sp [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;
}

// 【打印参数函数】
void pos_controller_UDE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>UDE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"Quad_MASS : "<< Quad_MASS << endl;

    cout <<"Kp_x : "<< Kp(0) << endl;
    cout <<"Kp_y : "<< Kp(1) << endl;
    cout <<"Kp_z : "<< Kp(2) << endl;

    cout <<"Kd_x : "<< Kd(0) << endl;
    cout <<"Kd_y : "<< Kd(1) << endl;
    cout <<"Kd_z : "<< Kd(2) << endl;

    cout <<"T_ude_x : "<< T_ude(0) << endl;
    cout <<"T_ude_y : "<< T_ude(1) << endl;
    cout <<"T_ude_z : "<< T_ude(2) << endl;

    cout <<"XY_VEL_MAX : "<< XY_VEL_MAX << endl;
    cout <<"Z_VEL_MAX : "<< Z_VEL_MAX << endl;

    cout <<"INT_LIM_X : "<< INT_LIM(0) << endl;
    cout <<"INT_LIM_Y : "<< INT_LIM(1) << endl;
    cout <<"INT_LIM_Z : "<< INT_LIM(2) << endl;

    cout <<"THR_MIN : "<< THR_MIN << endl;
    cout <<"THR_MAX : "<< THR_MAX << endl;

    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"throttle_a : "<< throttle_a << endl;

    cout <<"throttle_b : "<< throttle_b << endl;

}

}
#endif
