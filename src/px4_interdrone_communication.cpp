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

using std::vector;
using std::string;
struct SubTopic
{
    char str[100];
};
struct PubTopic
{
    char str[100];
};

int main(int argc, 
         char **argv) 
{
    ros::init(argc, argv, "px4_interdrone_communication");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    int numofdrones = 1;
    nh.param<int>("Pos_TCST/num_drone", numofdrones, 1);
/*------------sub */
/*----------- determine the  ID of the drone -----------------------------*/
    SubTopic mocap_UAV;
    SubTopic mavros_state;
    SubTopic mavros_imu_data;
    PubTopic px4_command_drone_state;
    PubTopic mavros_vision_pose_pose;



    // add preflex: (mavros and px4_command use lower case uav, while mocap use upper case UAV)
    strcpy (mocap_UAV.str,"/mocap/UAV"); 
    strcpy (mavros_state.str,"/uav"); 
    strcpy (mavros_imu_data.str,"/uav"); 
    strcpy (px4_command_drone_state.str,"/uav"); 
    strcpy (mavros_vision_pose_pose.str,"/uav"); 
    if ( argc > 1) {
    // if ID is specified as the second argument 
        strcat (mocap_UAV.str,argv[1]);
        strcat (mavros_state.str,argv[1]);
        strcat (mavros_imu_data.str,argv[1]);
        strcat (px4_command_drone_state.str,argv[1]);
        strcat (mavros_vision_pose_pose.str,argv[1]);
        ROS_INFO("UAV ID specified as: UAV%s", argv[1]);
    } else {
        // if ID is not specified, then set the drone to UAV0
        strcat (mocap_UAV.str,"0");
        strcat (mavros_state.str,"0");
        strcat (mavros_imu_data.str,"0");
        strcat (px4_command_drone_state.str,"0");
        strcat (mavros_vision_pose_pose.str,"0");
        ROS_WARN("NO UAV ID specified, set ID to 0.");
    }
    
    strcat (mavros_state.str,"/mavros/state");
    strcat (px4_command_drone_state.str,"/px4_command/drone_state");
    strcat (mavros_vision_pose_pose.str,"/mavros/vision_pose/pose");

    ROS_INFO("The total number of drones is: %d", numofdrones);
    ROS_INFO("Subscribe uav Mocap from: %s", mocap_UAV.str);
    ROS_INFO("Subscribe payload from: /mocap/Payload");
    ROS_INFO("Subscribe mavros_msgs::State from: %s", mavros_state.str);
    ROS_INFO("Subscribe IMU from: %s", mavros_imu_data.str); 



    ros::Subscriber MavrosState_sub    = nh.subscribe<mavros_msgs::State>(mavros_state.str, 100, GetMavrosState);
    ros::Subscriber Attitude_sub       = nh.subscribe<sensor_msgs::Imu>(mavros_imu_data.str, 100, GetAttitude);

    ROS_INFO("Start the interdrone communication...");

    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();


        rate.sleep();
    }
    return 0;
}
