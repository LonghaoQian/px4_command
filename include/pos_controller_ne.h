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
#include <math_utils.h>

#include <LowPassFilter.h>
#include <HighPassFilter.h>
#include <LeadLagFilter.h>


using namespace std;

namespace namespace_NE{

class pos_controller_NE
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller_NE(void):
            pos_NE_nh("~")
        {
            pos_NE_nh.param<float>("NE_MASS", NE_MASS, 1.0);
            pos_NE_nh.param<float>("NE_Kp_XY", NE_Kp(0), 1.0);
            pos_NE_nh.param<float>("NE_Kp_XY", NE_Kp(1), 1.0);
            pos_NE_nh.param<float>("NE_Kp_Z", NE_Kp(2), 1.0);
            pos_NE_nh.param<float>("NE_Kd_XY", NE_Kd(0), 2.0);
            pos_NE_nh.param<float>("NE_Kd_XY", NE_Kd(1), 2.0);
            pos_NE_nh.param<float>("NE_Kd_Z", NE_Kd(2), 2.0);
            pos_NE_nh.param<float>("NE_T_ude_XY", NE_T_ude(0), 1.0);
            pos_NE_nh.param<float>("NE_T_ude_XY", NE_T_ude(1), 1.0);
            pos_NE_nh.param<float>("NE_T_ude_Z", NE_T_ude(2), 1.0);
            pos_NE_nh.param<float>("NE_Tn", NE_Tn, 1.0);
            pos_NE_nh.param<float>("NE_INT_LIM_X", NE_INT_LIM(0), 1.0);
            pos_NE_nh.param<float>("NE_INT_LIM_Y", NE_INT_LIM(1), 1.0);
            pos_NE_nh.param<float>("NE_INT_LIM_Z", NE_INT_LIM(2), 5.0);

            pos_NE_nh.param<float>("NE_XY_VEL_MAX", NE_XY_VEL_MAX, 1.0);
            pos_NE_nh.param<float>("NE_Z_VEL_MAX", NE_Z_VEL_MAX, 1.0);

            pos_NE_nh.param<float>("NE_THR_MIN", NE_THR_MIN, 0.1);
            pos_NE_nh.param<float>("NE_THR_MAX", NE_THR_MAX, 0.9);

            pos_NE_nh.param<float>("NE_tilt_max", NE_tilt_max, 20.0);

            pos_NE_nh.param<float>("NE_a", NE_a, 20.0);
            pos_NE_nh.param<float>("NE_b", NE_b, 0.0);

            thrust_sp       = Eigen::Vector3d(0.0,0.0,0.0);
            u_l             = Eigen::Vector3d(0.0,0.0,0.0);
            u_d             = Eigen::Vector3d(0.0,0.0,0.0);
            u_total         = Eigen::Vector3d(0.0,0.0,0.0);
            integral_NE    = Eigen::Vector3d(0.0,0.0,0.0);
            integral_NE_LLF    = Eigen::Vector3d(0.0,0.0,0.0);
            error_pos       = Eigen::Vector3d(0.0,0.0,0.0);
            error_vel       = Eigen::Vector3d(0.0,0.0,0.0);
            NoiseEstimator  = Eigen::Vector3d(0.0,0.0,0.0);
            delta_time      = 0.0;
            flag_offboard   = 0;

            state_sub = pos_NE_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &pos_controller_NE::state_cb,this);

            set_filter();
        }


        //Mass of the quadrotor
        float NE_MASS;

        //NE control parameter
        Eigen::Vector3f NE_Kp;

        Eigen::Vector3f NE_Kd;

        Eigen::Vector3f NE_T_ude;

        float NE_Tn;

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

        //Limitation of NE integral
        Eigen::Vector3f NE_INT_LIM;

        //Limitation of the velocity
        float NE_XY_VEL_MAX;
        float NE_Z_VEL_MAX;

        //Limitation of the thrust
        float NE_THR_MIN;
        float NE_THR_MAX;

        //Limitation of the tilt angle (roll and pitch)  [degree]
        float NE_tilt_max;

        float NE_a;
        float NE_b;

        Eigen::Vector3d error_pos,error_vel;

        //u_l for nominal contorol(PD), u_d for NE control(disturbance estimator)
        Eigen::Vector3d u_l,u_d,u_total;

        Eigen::Vector3d integral_NE;

        Eigen::Vector3d integral_NE_LLF;

        //Desired thurst of the drone[the output of this class]
        Eigen::Vector3d thrust_sp;

        //The delta time between now and the last step
        float delta_time;

        //Current state of the drone
        mavros_msgs::State current_state;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;

        //Printf the NE parameter
        void printf_param();

        //Printf the control result
        void printf_result();

        //Position control main function [Input: current pos, current vel, desired state(pos or vel), sub_mode, time_now; Output: desired thrust;]
        Eigen::Vector3d pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, float dt);

        Eigen::Vector3d set_initial_pos(Eigen::Vector3d pos);

        void set_filter();

    private:

        ros::NodeHandle pos_NE_nh;

        ros::Subscriber state_sub;

        Eigen::Vector3d pos_initial;

        Eigen::Vector3d NoiseEstimator;

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

void pos_controller_NE::set_filter()
{
    LPF_x.set_Time_constant(NE_Tn);
    LPF_y.set_Time_constant(NE_Tn);
    LPF_z.set_Time_constant(NE_Tn);

    HPF_x.set_Time_constant(NE_Tn);
    HPF_y.set_Time_constant(NE_Tn);
    HPF_z.set_Time_constant(NE_Tn);

    LLF_x.set_Time_constant(NE_Tn, NE_Kd(0));
    LLF_y.set_Time_constant(NE_Tn, NE_Kd(1));
    LLF_z.set_Time_constant(NE_Tn, NE_Kd(2));
}

Eigen::Vector3d pos_controller_NE::set_initial_pos(Eigen::Vector3d pos)
{
    pos_initial = pos;
}


Eigen::Vector3d pos_controller_NE::pos_controller(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d pos_sp, float dt)
{
    delta_time = dt;

    error_pos = pos_sp - pos;
    error_vel = - vel;

    //Noise estimator
    NoiseEstimator(0) = LPF_x.apply(vel(0), delta_time) + HPF_x.apply(pos_initial(0) - pos(0), delta_time);
    NoiseEstimator(1) = LPF_y.apply(vel(1), delta_time) + HPF_y.apply(pos_initial(1) - pos(1), delta_time);
    NoiseEstimator(2) = LPF_z.apply(vel(2), delta_time) + HPF_z.apply(pos_initial(2) - pos(2), delta_time);

    //u_l
    for (int i = 0; i < 3; i++)
    {
       u_l(i) = NE_MASS * (NE_Kp(i) * error_pos(i) + NE_Kd(i) * ( error_vel(i) + NoiseEstimator(i)));
    }

    //UDE term
    Eigen::Vector3d input_LLF;

    for (int i = 0; i < 3; i++)
    {
        integral_NE_LLF(i) = integral_NE_LLF(i) +  vel(i) * delta_time;
        input_LLF(i) =  integral_NE_LLF(i) - pos(i) + pos_initial(i);
    }


    for (int i = 0; i < 3; i++)
    {
        integral_NE(i) = integral_NE(i) +  (NE_Kp(i) * error_pos(i) + NE_Kd(i) * error_vel(i)) * delta_time;
    }

    u_d(0) = NE_MASS /NE_T_ude(0) *( vel(0) - LLF_x.apply(input_LLF(0), delta_time) + integral_NE(0) );
    u_d(1) = NE_MASS /NE_T_ude(1) *( vel(1) - LLF_y.apply(input_LLF(1), delta_time) + integral_NE(1) );
    u_d(2) = NE_MASS /NE_T_ude(2) *( vel(2) - LLF_z.apply(input_LLF(2), delta_time) + integral_NE(2) );

    /* explicitly limit the integrator state */
    for (int i = 0; i < 3; i++)
    {
        u_d(i) = constrain_function2(u_d(i), -NE_INT_LIM(i), NE_INT_LIM(i));
    }

    //ENU frame
    u_total(0) = u_l(0) + u_d(0);
    u_total(1) = u_l(1) + u_d(1);
    u_total(2) = u_l(2) + u_d(2) + NE_MASS * 9.8;

    //Thrust to scale thrust[0,1]
    Eigen::Vector3d thrust_sp_scale;
    thrust_sp_scale(0) = (u_total(0) - NE_b) / NE_a;
    thrust_sp_scale(1) = (u_total(1) - NE_b) / NE_a;
    thrust_sp_scale(2) = (u_total(2) - NE_b) / NE_a;

    //Limit the Thrust
    thrust_sp(2) = constrain_function2( thrust_sp_scale(2) , NE_THR_MIN, NE_THR_MAX);

    // Get maximum allowed thrust in XY based on tilt angle and excess thrust.
    float thrust_max_XY_tilt = fabs(thrust_sp(2)) * tanf(NE_tilt_max/180.0*M_PI);
    float thrust_max_XY = sqrtf(NE_THR_MAX * NE_THR_MAX - thrust_sp(2) * thrust_sp(2));
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
        integral_NE = Eigen::Vector3d(0.0,0.0,0.0);
    }

    return thrust_sp;
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

    cout << "delta_time : " << delta_time<< " [s] " <<endl;

    cout << "NoiseEstimator [X Y Z] : " << NoiseEstimator[0] << " [N] "<< NoiseEstimator[1]<<" [N] "<<NoiseEstimator[2]<<" [N] "<<endl;
    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;

    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;

    cout << "u_total [X Y Z] : " << u_total[0] << " [N] "<< u_total[1]<<" [N] "<<u_total[2]<<" [N] "<<endl;

    cout << "thrust_sp [X Y Z] : " << thrust_sp[0] << " [m/s^2] "<< thrust_sp[1]<<" [m/s^2] "<<thrust_sp[2]<<" [m/s^2] "<<endl;
}

// 【打印参数函数】
void pos_controller_NE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>NE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"NE_MASS : "<< NE_MASS << endl;

    cout <<"NE_Kp_X : "<< NE_Kp(0) << endl;
    cout <<"NE_Kp_Y : "<< NE_Kp(1) << endl;
    cout <<"NE_Kp_Z : "<< NE_Kp(2) << endl;

    cout <<"NE_Kd_X : "<< NE_Kd(0) << endl;
    cout <<"NE_Kd_Y : "<< NE_Kd(1) << endl;
    cout <<"NE_Kd_Z : "<< NE_Kd(2) << endl;

    cout <<"NE_T_X : "<< NE_T_ude(0) << endl;
    cout <<"NE_T_Y : "<< NE_T_ude(1) << endl;
    cout <<"NE_T_Z : "<< NE_T_ude(2) << endl;
    cout <<"NE_Tn : "<< NE_Tn << endl;

    cout <<"NE_XY_VEL_MAX : "<< NE_XY_VEL_MAX << endl;
    cout <<"NE_Z_VEL_MAX : "<< NE_Z_VEL_MAX << endl;

    cout <<"NE_INT_LIM_X : "<< NE_INT_LIM(0) << endl;
    cout <<"NE_INT_LIM_Y : "<< NE_INT_LIM(1) << endl;
    cout <<"NE_INT_LIM_Z : "<< NE_INT_LIM(2) << endl;

    cout <<"NE_THR_MIN : "<< NE_THR_MIN << endl;
    cout <<"NE_THR_MAX : "<< NE_THR_MAX << endl;

    cout <<"NE_tilt_max : "<< NE_tilt_max << endl;
    cout <<"NE_a : "<< NE_a << endl;

    cout <<"NE_b : "<< NE_b << endl;

    cout <<"Filter_LPFx : "<< LPF_x.get_Time_constant() << endl;

    cout <<"Filter_LLFx : "<< LLF_x.get_Kd() << endl;


}

}
#endif
