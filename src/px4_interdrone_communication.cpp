/***************************************************************************************************************************
 * px4_interdrone_communication.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2019.10.08
 * Get the control force and quadrotor relative state for robust cooperative payload control 
 * if the drone is uav0 
 * subscribe the /uav#/px4_command/control_output (f_Lj)
 * subscribe  
 * publish Delta_T and Delta_R 
 * if the drone is not uav0 
 * publish f_Lj and delta_j
 * subscribe Delta_T and Delta_R from uav0 
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <math_utils.h>
#include <mavros_msgs/State.h>
#include <Frame_tf_utils.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <px4_command/DroneState.h>
#include <px4_command/Mocap.h>
#include <px4_command/AddonForce.h>
#include <px4_command/FleetStatus.h>
using std::vector;
using std::string;

px4_command::FleetStatus uav0_status;
px4_command::FleetStatus uav1_status;
px4_command::FleetStatus uav2_status;
Eigen::Vector3f t_0;
Eigen::Vector3f t_1;
Eigen::Vector3f t_2;
Eigen::Vector3f Delta_T_I;
Eigen::Vector3f Delta_R_I;
Eigen::Vector3f Delta_T;
Eigen::Vector3f Delta_R;
Eigen::Vector3f Delta_rt;
Eigen::Vector3f Delta_pt;
void GetUAV0Status(const px4_command::FleetStatus::ConstPtr& msg){
    uav0_status = *msg;
}
void GetUAV1Status(const px4_command::FleetStatus::ConstPtr& msg){
    uav1_status = *msg;
}
void GetUAV2Status(const px4_command::FleetStatus::ConstPtr& msg){
    uav2_status = *msg;
}

void PrintEstimation(){

}

int main(int argc, 
         char **argv) 
{
    ros::init(argc, argv, "px4_interdrone_communication");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    int num_of_drones = 1;
    float payload_mass = 0;
    float total_quadmass = 0;
    float lambda_T = 0;
    float lambda_R = 0;

    nh.param<int>("Pos_GNC/num_drone", num_of_drones,1);

    for (int i = 0; i < num_of_drones ; i ++) {
        // reset names:
        temp_uav_pref = "uav";
        temp_uav_pref = temp_uav_pref + to_string(i);
        main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_x", temp_t_j(0), 0.5);
        main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_y", temp_t_j(1), 0);
        main_handle.param<float>(temp_uav_pref+"_Pos_GNC/TetherOffset_z", temp_t_j(2), 0); 
        main_handle.param<float>(temp_uav_pref+"_Pos_GNC/PayloadSharingPortion", temp_a_j, 0); 
        D += temp_a_j * Hatmap(temp_t_j)* Hatmap(temp_t_j);
    }
    ros::Subscriber subUAV0status = nh.subscribe<px4_command::FleetStatus>("/uav0/px4_command/fleetstatus", 100, GetUAV0Status);
    ros::Subscriber subUAV1status = nh.subscribe<px4_command::FleetStatus>("/uav1/px4_command/fleetstatus", 100, GetUAV1Status);
    ros::Subscriber subUAV2status = nh.subscribe<px4_command::FleetStatus>("/uav2/px4_command/fleetstatus", 100, GetUAV2Status);
    ros::Subscriber subDronestate = nh.subscribe<px4_command::DroneState> ("/uav0/px4_command/dronestate", 100, GetUAV3Status);
    ros::Publisher  pubAddonForce = nh.advertise<px4_command::AddonForce> ("/uav0/px4_command/addonforce", 1000);

/*--------Print Parameters-----------*/


    px4_command::AddonForce    _AddonForce;
    Eigen::Vector3f Deltfloat32  a_T;
    Eigen::Vector3f Deltfloat32  a_pt;
    Eigen::Vector3f Deltfloat32  a_R;
    Eigen::Vector3f Deltfloat32  a_rt;
    Eigen::Vector3f A1;
    Eigen::Vector3f F1;
    Eigen::Vector3f A2;
    Eigen::Vector3f F2;
    Eigen::Vector3f g_I;

    Delta_T_I<< 0.0,
                0.0,
                0.0;
    Delta_R_I<< 0.0,
                0.0,
                0.0;
    Delta_T << 0.0,
               0.0,
               0.0;
    Delta_R << 0.0,
               0.0,
               0.0;
    Delta_pt << 0.0,
                0.0,
                0.0;
    Delta_rt << 0.0,
               0.0,
               0.0;
    g_I << 0.0,
           0.0,
         -9.81;
    ROS_INFO("Start the interdrone communication and disturbance estimation...");
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0.0;
    while(ros::ok())
    {
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        ros::spinOnce();




        A1 << 0.0,
              0.0,
              0.0; 
        F1 << 0.0,
              0.0,
              0.0;
        for (int i = 0; i < num_of_drones ; i ++) {


           A1+= B_j * v_j;
           F1+= f_L_j;

           F2+= 

        }

        if(_DroneState.mode != "OFFBOARD") {
            Delta_T<< 0.0,
                      0.0,
                      0.0;
            Delta_R<< 0.0,
                      0.0,
                      0.0;
            Delta_T_I<< 0.0,
                        0.0,
                        0.0;
            Delta_R_I<< 0.0,
                        0.0,
                         0.0;
            Delta_pt<< 0.0,
                       0.0,
                       0.0;
            Delta_rt<< 0.0,
                       0.0,
                       0.0;
        } else {
            

            Delta_T_I += dt* ( Delta_T + F1 + (M_q + m_p) * g_I);
        
        }


        Delta_T = lambda_T * ( - Delta_T_I + (M_q + m_p) * g_I + R_IP A.transpose() * omega_p +  A1);

        Delta_R = lambda_R * ();

        _AddonForce.delta_Tx = Delta_pt(0);
        _AddonForce.delta_Ty = Delta_pt(1);
        _AddonForce.delta_Tz = Delta_pt(2);  

        _AddonForce.delta_Rx = Delta_rt(0);
        _AddonForce.delta_Ry = Delta_rt(1);  
        _AddonForce.delta_Rz = Delta_rt(2); 

        _AddonForce.R_1x = 0;
        _AddonForce.R_1y = 0;
        _AddonForce.R_1z = 0;

        _AddonForce.R_2x = 0;
        _AddonForce.R_2y = 0;
        _AddonForce.R_2z = 0;

        pubAddonForce.publish(_AddonForce);
        rate.sleep();
    }
    return 0;
}
