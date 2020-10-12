/***************************************************************************************************************************
 * px4_multidrone_pos_estimator_outdoor.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2020.10.11
 *
 * Get drone position, velocity and payload visual measurement from gps and camera
 *      1. subscrbe the filtered drone position and velocity from ekf
 *      2. publish the drone velocity and position and payload estimation 
 *
***************************************************************************************************************************/
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Eigen>

#include <state_from_mavros_multidrone.h>
#include <math_utils.h>
#include <px4_command/DroneState.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

void GetGPSPos(const nav_msgs::Odometry::ConstPtr& msg, px4_command::DroneState* dronestate){
    dronestate->position[math_utils::Vector_X] = msg->pose.pose.position.x;
    dronestate->position[math_utils::Vector_Y] = msg->pose.pose.position.y;
    dronestate->position[math_utils::Vector_Z] = msg->pose.pose.position.z;
    dronestate->velocity[math_utils::Vector_X] = msg->twist.twist.linear.x;
    dronestate->velocity[math_utils::Vector_Y] = msg->twist.twist.linear.y;
    dronestate->velocity[math_utils::Vector_Z] = - msg->twist.twist.linear.z;
}

void GetUAVBattery(const sensor_msgs::BatteryState::ConstPtr &msg, px4_command::DroneState* dronestate) {
    dronestate->battery_voltage   = msg->voltage;
    dronestate->battery_remaining = msg->percentage;
}

void GetMavrosState(const mavros_msgs::State::ConstPtr &msg, px4_command::DroneState* dronestate) {
    dronestate->connected = msg->connected;
    dronestate->armed = msg->armed;
    dronestate->mode = msg->mode;
}

void GetDroneAttitude(const sensor_msgs::Imu::ConstPtr& msg,  px4_command::DroneState* _DroneState) {
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    _DroneState->attitude_q.w = q_fcu.w();
    _DroneState->attitude_q.x = q_fcu.x();
    _DroneState->attitude_q.y = q_fcu.y();
    _DroneState->attitude_q.z = q_fcu.z();
    _DroneState->attitude[math_utils::EULER_ROLL]  = euler_fcu[math_utils::EULER_ROLL];
    _DroneState->attitude[math_utils::EULER_PITCH] = euler_fcu[math_utils::EULER_PITCH];
    _DroneState->attitude[math_utils::EULER_YAW]   = euler_fcu[math_utils::EULER_YAW];
    _DroneState->attitude_rate[math_utils::Vector_X] = msg->angular_velocity.x;
    _DroneState->attitude_rate[math_utils::Vector_Y] = msg->angular_velocity.y;
    _DroneState->attitude_rate[math_utils::Vector_Z] = msg->angular_velocity.z;
    _DroneState->acceleration[math_utils::Vector_X] = msg->linear_acceleration.x;
    _DroneState->acceleration[math_utils::Vector_Y] = msg->linear_acceleration.y;
    _DroneState->acceleration[math_utils::Vector_Z] = msg->linear_acceleration.z;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "px4_multidrone_pos_estimator_outdoor");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    // define variables
    namespace arg = std::placeholders;
    px4_command::DroneState DroneStateData;  
    bool IsPayloadVisionReceived = false;
    // define the drone ID and ros topics:
    std::string TopicMavrosGPSPos = "/uav";
    std::string TopicVisionPayload = "/uav";
    std::string TopicPx4CommandDroneState = "/uav";
    std::string TopicMavrosIMU = "/uav";
    std::string TopicMavrosState = "/uav";
    std::string TopicMavrosBattery = "/uav";

    char* droneID = NULL;
    char droneDefaultID = '0';
    if ( argc > 1) {    // if ID is specified as the second argument 
        ROS_INFO("UAV ID specified as: UAV%s", argv[1]);
        droneID = argv[1];
    } else {
        // if ID is not specified, then set the drone to UAV0
        droneID = &droneDefaultID;
        ROS_WARN("NO UAV ID is specified, set the ID to 0!");
    }
    // add uav prefixes to topic strings 
    TopicMavrosGPSPos.push_back(*droneID);
    TopicVisionPayload.push_back(*droneID);
    TopicPx4CommandDroneState.push_back(*droneID);
    TopicMavrosIMU.push_back(*droneID);
    TopicMavrosState.push_back(*droneID);
    TopicMavrosBattery.push_back(*droneID);
    // add topic names to topic strings
    TopicMavrosGPSPos += "/mavros/global_position/local";
    //TopicVisionPayload.push_back()
    TopicPx4CommandDroneState += "/px4_command/drone_state";
    TopicMavrosIMU += "/mavros/imu/data";
    TopicMavrosState += "/mavros/state";
    TopicMavrosBattery += "/mavros/battery";
    // define publishers and subscribers
    ros::Publisher  PubDroneState = nh.advertise<px4_command::DroneState>(TopicPx4CommandDroneState, 100);
    ros::Subscriber SubGPSPosition    = nh.subscribe<nav_msgs::Odometry>(TopicMavrosGPSPos, 
                                                                         100, 
                                                                         std::bind(&GetGPSPos, arg::_1, &DroneStateData));

    ros::Subscriber SubMavrosBattery  = nh.subscribe<sensor_msgs::BatteryState> (TopicMavrosBattery,
                                                                                 100,
                                                                                 std::bind(&GetUAVBattery, arg::_1, &DroneStateData));

    ros::Subscriber SubMavrosState    = nh.subscribe<mavros_msgs::State>(TopicMavrosState, 
                                                                         100, 
                                                                         std::bind(&GetMavrosState, arg::_1,  &DroneStateData));

    ros::Subscriber SubDroneAttitude  = nh.subscribe<sensor_msgs::Imu>(TopicMavrosIMU, 
                                                                       100, 
                                                                       std::bind(&GetDroneAttitude, arg::_1,  &DroneStateData));
    // display topics:
    ROS_INFO("Subscribe uav GPS position and velocity from: %s", TopicMavrosGPSPos.c_str());
    ROS_INFO("Subscribe mavros_msgs::State from: %s", TopicMavrosState.c_str());
    ROS_INFO("Subscribe IMU from: %s", TopicMavrosIMU.c_str()); 
    ROS_INFO("Subscribe battery info from: %s", TopicMavrosBattery.c_str()); 
    ROS_INFO("Publish DroneState to: %s", TopicPx4CommandDroneState.c_str());
    ROS_INFO("Start the estimator...");
    while(ros::ok()) {
        ros::spinOnce();
        DroneStateData.header.stamp = ros::Time::now();
        PubDroneState.publish(DroneStateData);
        rate.sleep();
    }

    return 0;
}