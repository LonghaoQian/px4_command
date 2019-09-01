/***************************************************************************************************************************
* state_from_mavros_multidrone.h
*
* Author: Longhao Qian
*
* Update Time: 2019.9.1
*
* 主要功能：
*    本库函数主要用于连接px4_command与mavros两个功能包。 for multi_drone case
*
***************************************************************************************************************************/
#ifndef STATE_FROM_MAVROS_MULTIDRONE_H
#define STATE_FROM_MAVROS_MULTIDRONE_H

#include <ros/ros.h>
#include <math_utils.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <px4_command/DroneState.h>
#include <bitset>
#include <px4_command/AttitudeReference.h>
#include <px4_command/DroneState.h>

using namespace std;

class state_from_mavros_multidrone
{
    public:
    //constructed function
    state_from_mavros_multidrone(const char* ID): // add drone ID as input argument
        state_nh("~")
    {
        
        char mavros_state[100];//
        char mavros_local_position_pose[100];//
        char mavros_local_position_velocity_local[100];//
        char mavros_imu_data[100];//

        strcpy (mavros_state,"/uav"); 
        strcpy (mavros_local_position_pose,"/uav"); 
        strcpy (mavros_local_position_velocity_local,"/uav"); 
        strcpy (mavros_imu_data,"/uav"); 

        strcat (mavros_state, ID);
        strcat (mavros_local_position_pose,ID);
        strcat (mavros_local_position_velocity_local,ID);
        strcat (mavros_imu_data,ID);

        strcat (mavros_state, "/mavros/state");
        strcat (mavros_local_position_pose,"/mavros/local_position/pose");
        strcat (mavros_local_position_velocity_local,"/mavros/local_position/velocity_local");
        strcat (mavros_imu_data,"/mavros/imu/data");

        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = state_nh.subscribe<mavros_msgs::State>(mavros_state, 10, &state_from_mavros_multidrone::state_cb,this);

        // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        position_sub = state_nh.subscribe<geometry_msgs::PoseStamped>(mavros_local_position_pose, 10, &state_from_mavros_multidrone::pos_cb,this);

        // 【订阅】无人机当前速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        velocity_sub = state_nh.subscribe<geometry_msgs::TwistStamped>(mavros_local_position_velocity_local, 10, &state_from_mavros_multidrone::vel_cb,this);

        // 【订阅】无人机当前欧拉角 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        attitude_sub = state_nh.subscribe<sensor_msgs::Imu>(mavros_imu_data, 10, &state_from_mavros_multidrone::att_cb,this);

        ROS_INFO("Subscribe mavros State from: %s", mavros_state);
        ROS_INFO("Subscribe mavros local position pose from: %s ", mavros_local_position_pose);
        ROS_INFO("Subscribe mavros local velocity from: %s", mavros_local_position_velocity_local);
        ROS_INFO("Subscribe IMU from: %s", mavros_imu_data); 
    }

    //变量声明 
    px4_command::DroneState _DroneState;

    private:

        ros::NodeHandle state_nh;

        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            _DroneState.connected = msg->connected;
            _DroneState.armed = msg->armed;
            _DroneState.mode = msg->mode;
        }

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            _DroneState.position[0] = msg->pose.position.x;
            _DroneState.position[1] = msg->pose.position.y;
            _DroneState.position[2] = msg->pose.position.z;
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            _DroneState.velocity[0] = msg->twist.linear.x;
            _DroneState.velocity[1] = msg->twist.linear.y;
            _DroneState.velocity[2] = msg->twist.linear.z;
        }

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
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


};

    
#endif
