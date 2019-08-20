/***************************************************************************************************************************
 * px4_estimator_vision.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2019.8.20
 *
 * 说明: mavros位置估计程序 (optitrack only)
 *      1. subscribe position and velocity data of drone and payload from ground station via Mocap topic
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
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

using namespace std;
px4_command::DroneState _DroneState;  
px4_command::Mocap UAV_motion;
px4_command::Mocap Payload_motion;
bool UAVsubFlag;
bool PaylaodsubFlag;
bool MocapOK;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void GetUAVState(const px4_command::Mocap::ConstPtr& msg) {
    UAV_motion = *msg;
    for ( int i = 0; i < 3; i ++) {
        _DroneState.position[i] = UAV_motion.position[i];
        _DroneState.velocity[i] = UAV_motion.velocity[i];            
    }
    UAVsubFlag = true;
}
void GetPayloadState(const px4_command::Mocap::ConstPtr& msg) {
    Payload_motion = *msg;
    for ( int i = 0; i < 3; i ++) {
        _DroneState.payload_vel[i] = Payload_motion.velocity[i];
        _DroneState.payload_pos[i] = Payload_motion.position[i];              
    }
    PaylaodsubFlag = true;
}
void GetMavrosState(const mavros_msgs::State::ConstPtr &msg) {
    _DroneState.connected = msg->connected;
    _DroneState.armed = msg->armed;
    _DroneState.mode = msg->mode;
}
void GetAttitude(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    _DroneState.attitude_q.w = q_fcu.w();
    _DroneState.attitude_q.x = q_fcu.x();
    _DroneState.attitude_q.y = q_fcu.y();
    _DroneState.attitude_q.z = q_fcu.z();
    _DroneState.attitude[0] = euler_fcu[0];
    _DroneState.attitude[1] = euler_fcu[1];
    _DroneState.attitude[2] = euler_fcu[2];
    _DroneState.attitude_rate[0] = msg->angular_velocity.x;
    _DroneState.attitude_rate[1] = msg->angular_velocity.x;
    _DroneState.attitude_rate[2] = msg->angular_velocity.x;
}
int main(int argc, 
         char **argv) {
    ros::init(argc, argv, "px4_pos_estimator_pure_vision");
    ros::NodeHandle nh("~");
    // subscriber
    ros::Subscriber UAV_motion_sub     = nh.subscribe<px4_command::Mocap>("/mocap/UAV", 1000, GetUAVState);
    ros::Subscriber Payload_motion_sub = nh.subscribe<px4_command::Mocap>("/mocap/Payload", 1000, GetPayloadState);
    ros::Subscriber MavrosState_sub    = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, GetMavrosState);
    ros::Subscriber Attitude_sub       = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, GetAttitude);
    // publisher
    ros::Publisher drone_state_pub = nh.advertise<px4_command::DroneState>("/px4_command/drone_state", 100);

    // ROS frequency
    ros::Rate rate(50.0);

    const int MocapTolerance = 3;
    int NoFeedBackCounter = 0;
    
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();
        _DroneState.header.stamp = ros::Time::now();
        MocapOK = true; // reset the mocap flag to false
        if ( !UAVsubFlag ) {
            MocapOK = false;
        }
        if ( !PaylaodsubFlag ) {
            MocapOK = false;
        }
        // reset the sub flag to false
        UAVsubFlag = false;
        PaylaodsubFlag = false;
        // determine whether the mocap feedback is normal
        _DroneState.mocapOK = true;
        if (MocapOK) {
            NoFeedBackCounter = 0;
            _DroneState.mocapOK = true;
        } else {
            NoFeedBackCounter++;
            if ( NoFeedBackCounter >= MocapTolerance ) {
                _DroneState.mocapOK = false;
            } else {
                _DroneState.mocapOK = true;
            }
        }
        drone_state_pub.publish(_DroneState);
        rate.sleep();
    }
    return 0;
}



