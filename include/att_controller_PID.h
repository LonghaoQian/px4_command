/***************************************************************************************************************************
* att_controller_PID.h
*
* Author: Qyp
*
* Update Time: 2019.3.23
*
* Introduction:  attition Controller using PID (P for att loop, pid for rates loop)
*         1. Similiar to the attition controller in PX4 (1.8.2)
*         2. Ref to : https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/attitionControl.cpp
*         3. Here we didn't consider the mass of the drone, we treat accel_sp is the thrust_sp.
*         4. We didn't use filter to get the derivation of the ratesocity [It must be considered.]
*         5. thrustToAttitude ref to https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/Utility/ControlMath.cpp
***************************************************************************************************************************/
#ifndef ATT_CONTROLLER_PID_H
#define ATT_CONTROLLER_PID_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

#include <mavros_msgs/ActuatorControl>


using namespace std;

namespace namespace_PID {

class att_controller_PID
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        att_controller_PID(void):
            pid_nh("~")
        {
            pid_nh.param<float>("MC_ROLL_P", MC_ROLL_P, 6.5);
            pid_nh.param<float>("MC_ROLLRATE_P", MC_ROLLRATE_P, 0.15);
            pid_nh.param<float>("MC_ROLLRATE_I", MC_ROLLRATE_I, 0.05);


            Euler_fcu       = Eigen::Vector3d(0.0,0.0,0.0);
            q_fcu           = Eigen::Quaterniond(0.0,0.0,0.0,0.0);

            angular_ratesocity = Eigen::Vector3d(0.0,0.0,0.0);


            flag_offboard   = 0;

            state_sub = pid_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &att_controller_PID::state_cb,this);
            attitude_sub = command_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &command_to_mavros::att_cb,this);


        }

        //PID parameter for the control law
        float MC_ROLL_P;
        float MC_ROLLRATE_P;
        float MC_ROLLRATE_I;
        float MC_ROLLRATE_D;

        float MC_PITCH_P;
        float MC_PITCHRATE_P;
        float MC_PITCHRATE_I;
        float MC_PITCHRATE_D;

        float MC_YAW_P;
        float MC_YAWRATE_P;
        float MC_YAWRATE_I;
        float MC_YAWRATE_D;

        float MC_ROLLRATE_MAX;
        float MC_PITCHRATE_MAX;
        float MC_YAWRATE_MAX;

        float MC_RR_INT_LIM;
        float MC_PR_INT_LIM;
        float MC_YR_INT_LIM;



        //Current state of the drone
        mavros_msgs::State current_state;

        //Flag of the offboard mode [1 for OFFBOARD mode , 0 for non-OFFBOARD mode]
        int flag_offboard;


        //Current att of the drone
        Eigen::Quaterniond q_fcu;
        Eigen::Vector3d Euler_fcu;
        Eigen::Vector3d rates;

        //
        Eigen::Vector3d euler_sp;
        Eigen::Vector3d rates_sp;


        //Output of the rates loop in PID [rates_int is the I]
        Eigen::Vector3d rates_P_output;
        Eigen::Vector3d rates_int;
        Eigen::Vector3d rates_D_output;

        //The delta time between now and the last step
        float delta_time;
        //Time of the last step
        float last_time;

        //Error of the rates in last step [used for the D-output in rates loop]
        Eigen::Vector3d error_rates_last;


        float thrust_sp;
        mavros_msgs::ActuatorControl actuator_setpoint;



        //Printf the PID parameter
        void printf_pid_param();

        //Printf the control result
        void printf_result();

        //attition control main function [Input: desired state, time_now; Output: actuator_setpoint;]
        void att_controller(Eigen::Vector3d att_sp, float thr_sp_body, float curtime);

        //attition control loop [Input: desired state; Output: desired rates]
        void _attController(Eigen::Vector3d att_sp);

        //ratesocity control loop [Input: rates_sp; Output: actuator_setpoint]
        void _attrateController();

    private:

        ros::NodeHandle pid_nh;

        ros::Subscriber state_sub;
        ros::Subscriber attitude_sub;

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

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to Euler Angles
            Euler_fcu = quaternion_to_euler(q_fcu);

            rates = Eigen::Vector3d(msg->angular_ratesocity.x, msg->angular_ratesocity.y, msg->angular_ratesocity.z);

        }


};


void att_controller_PID::att_controller(Eigen::Vector3d att_sp, float thr_sp_body, float curtime)
{
    delta_time = curtime - last_time;

    thrust_sp = thr_sp_body;

    _attController(att_sp);

    _attrateController();

    printf_result();

    last_time = curtime;

}


void att_controller_PID::_attController(Eigen::Vector3d att_sp)
{
    rates_sp(0) = MC_ROLL_P * (att_sp(0) - Euler_fcu(0));
    rates_sp(1) = MC_PITCH_P * (att_sp(1) - Euler_fcu(1));
    rates_sp(2) = MC_YAW_P  * (att_sp(2) - Euler_fcu(2));

    // Limit the ratesocity setpoint
    rates_sp(0) = constrain_function2(rates_sp(0), -MC_ROLLRATE_MAX, MC_ROLLRATE_MAX);
    rates_sp(1) = constrain_function2(rates_sp(1), -MC_PITCHRATE_MAX, MC_PITCHRATE_MAX);
    rates_sp(2) = constrain_function2(rates_sp(2), -MC_YAWRATE_MAX, MC_YAWRATE_MAX);
}

void att_controller_PID::_attrateController()
{
    Eigen::Vector3d error_rates = rates_sp - rates;

    rates_P_output(0) = MC_ROLLRATE_P * error_rates(0);
    rates_P_output(1) = MC_PITCHRATE_P * error_rates(1);
    rates_P_output(2) = MC_YAWRATE_P  * error_rates(2);

    rates_D_output(0) = MC_ROLLRATE_D * (error_rates(0) - error_rates_last(0)) / delta_time;
    rates_D_output(1) = MC_PITCHRATE_D * (error_rates(1) - error_rates_last(1)) / delta_time;
    rates_D_output(2) = MC_YAWRATE_D  * (error_rates(2) - error_rates_last(2)) / delta_time;

    // Update integral
    rates_int(0) += MC_ROLLRATE_I * error_rates(0) * delta_time;
    rates_int(1) += MC_PITCHRATE_I * error_rates(1) * delta_time;
    rates_int(2) += MC_YAWRATE_I * error_rates(1) * delta_time;

    rates_int(0) = constrain_function2(rates_int(0), -MC_RR_INT_LIM, MC_RR_INT_LIM);
    rates_int(1) = constrain_function2(rates_int(1), -MC_PR_INT_LIM, MC_PR_INT_LIM);
    rates_int(2) = constrain_function2(rates_int(2), -MC_YR_INT_LIM, MC_YR_INT_LIM);


    //
    actuator_setpoint.controls[0]  = rates_P_output(0) + rates_int(0) + rates_D_output(0);
    actuator_setpoint.controls[1]  = rates_P_output(1) + rates_int(1) + rates_D_output(1);
    actuator_setpoint.controls[2]  = rates_P_output(2) + rates_int(2) + rates_D_output(2);
    actuator_setpoint.controls[3] = thrust_sp;

    error_rates_last = error_rates;

    //If not in OFFBOARD mode, set all intergral to zero.
    if(flag_offboard == 0)
    {
        rates_int = Eigen::Vector3d(0.0,0.0,0.0);
    }
}



void att_controller_PID::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>attition Controller<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout.setf(ios::fixed);

    cout << "delta_time : " << fixed <<setprecision(3)<< delta_time<< " [s] " <<endl;

    cout << "euler_sp    [X Y Z] : " << euler_sp[0] / M_PI *180 << " [deg] "<< euler_sp[1] / M_PI *180 <<" [deg] "<<euler_sp[2]/ M_PI *180 <<" [deg] "<<endl;

    cout << "rates_sp    [X Y Z] : " << rates_sp[0] / M_PI *180 << " [deg/s] "<< rates_sp[1] / M_PI *180 <<" [deg/s] "<<rates_sp[2]/ M_PI *180 <<" [deg/s] "<<endl;

    cout << "rates_P_output [X Y Z] : " << rates_P_output[0] << " [m/s] "<< rates_P_output[1]<<" [m/s] "<<rates_P_output[2]<<" [m/s] "<<endl;

    cout << "rates_I_output [X Y Z] : " << rates_int[0] << " [m/s] "<< rates_int[1]<<" [m/s] "<<rates_int[2]<<" [m/s] "<<endl;

    cout << "rates_D_output [X Y Z] : " << rates_D_output[0] << " [m/s] "<< rates_D_output[1]<<" [m/s] "<<rates_D_output[2]<<" [m/s] "<<endl;

    cout << "actuator_setpoint [0 1 2] : " << actuator_setpoint.controls[0] << " [m/s^2] "<< actuator_setpoint.controls[1]<<" [m/s^2] "<<actuator_setpoint.controls[2]<<" [m/s^2] "<<endl;

    cout << "actuator_setpoint    [3] : " << actuator_setpoint.controls[3] <<endl;
}

// 【打印参数函数】
void att_controller_PID::printf_pid_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout <<"attition Loop:  " <<endl;
    cout <<"MPC_XY_P : "<< MPC_XY_P << endl;
    cout <<"MPC_Z_P : "<< MPC_Z_P << endl;
    cout <<"ratesocity Loop:  " <<endl;
    cout <<"MPC_XY_rates_P : "<< MPC_XY_rates_P << endl;
    cout <<"MPC_Z_rates_P : "<< MPC_Z_rates_P << endl;
    cout <<"MPC_XY_rates_I : "<< MPC_XY_rates_I << endl;
    cout <<"MPC_Z_rates_I : "<< MPC_Z_rates_I << endl;
    cout <<"MPC_XY_rates_D : "<< MPC_XY_rates_D << endl;
    cout <<"MPC_Z_rates_D : "<< MPC_Z_rates_D << endl;

    cout <<"Limit:  " <<endl;
    cout <<"MPC_XY_rates_MAX : "<< MPC_XY_rates_MAX << endl;
    cout <<"MPC_Z_rates_MAX : "<< MPC_Z_rates_MAX << endl;


    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"MPC_THR_MIN : "<< MPC_THR_MIN << endl;
    cout <<"MPC_THR_MAX : "<< MPC_THR_MAX << endl;
    cout <<"MPC_THRUST_HOVER : "<< MPC_THRUST_HOVER << endl;





}



}
#endif
