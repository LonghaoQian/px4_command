/***************************************************************************************************************************
* command_to_mavros.h
*
* Author: Qyp
*
* Update Time: 2019.6.28
*
* Introduction:  Drone control command send class using Mavros package
*         1. Ref to the Mavros plugins (setpoint_raw, loca_position, imu and etc..)
*         2. Ref to the Offboard Flight task in PX4 code: https://github.com/PX4/Firmware/blob/master/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp
*         3. Ref to the Mavlink module in PX4 code: https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp
*         4. Ref to the position control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control
*         5. Ref to the attitude control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control
*         6. 还需要考虑复合形式的输出情况
* 主要功能：
*    本库函数主要用于连接px4_command与mavros两个功能包。简单来讲，本代码提供飞控的状态量，用于控制或者监控，本代码接受控制指令，并将其发送至飞控。
* 1、发布px4_command功能包生成的控制量至mavros功能包，可发送期望位置、速度、角度、角速度、底层控制等。
* 2、订阅mavros功能包发布的飞控状态量（包括PX4中的期望位置、速度、角度、角速度、底层控制），用于检查飞控是否正确接收机载电脑的指令
* 3、解锁上锁、修改模式两个服务。
***************************************************************************************************************************/
#ifndef OPTITRACK_FILTER_H
#define OPTITRACK_FILTER_H

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






#pragma once
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
//#include "UtilityFunctions.h"
#include <Eigen/Eigen>
//maximum window size
#ifndef max_windowsize
#define max_windowsize 10
#endif
using namespace Eigen;

struct optitrack_pose{
    Vector3d Position;
    double q0;
    double q1;
    double q2;
    double q3;
    double t;
    Matrix<double, 3, 4> L;// Quaternion auxiliary matirx 
    Matrix<double, 3, 4> R;// Quaternion auxiliary matirx 
    Matrix3d R_IB; //
    Matrix3d R_BI; //
};

struct rigidbody_state{
    Vector4d quaterion;
    Vector3d Position;// inertial position
    Vector3d V_I; // inertial velocity
    Matrix3d Omega_Cross; // angular velocity skew
    Vector3d Omega_BI;// Frame B to Frame I expressed in Frame B
    Matrix3d R_IB; // rotation matrix
    Matrix3d R_BI; //
    Vector3d Euler;// euler angle
    double time_stamp;
};

class OptiTrackFeedBackRigidBody{

    //-------Optitrack Related-----///
    geometry_msgs::PoseStamped OptiTrackdata;
    unsigned int OptiTrackFlag; // OptiTrackState 0: no data feed,: 1 data feed present
    void OptiTrackCallback(const geometry_msgs::PoseStamped& msg);   
    unsigned int FeedbackState;// 0 no feedback, 1 has feedback
    ros::Subscriber subOptiTrack;// OptiTrack Data
    //--------Filter Parameters-------//
    unsigned int linear_velocity_window; // window size
    unsigned int angular_velocity_window; // window size
    //--------Filter Buffer-----------//
    // raw velocity buffer from numerical differentiation
    Vector3d  velocity_raw[max_windowsize];
    Vector3d  angular_velocity_raw[max_windowsize];
    Vector3d  velocity_filtered;        // filtered velocity
    Vector3d  angular_velocity_filtered;// filtered angular velocity
    optitrack_pose  pose[2];/*pose info from optitrack pose[1] should be the latest mesaured value, 
    pose[0] is value of the last measurment (in world frame by default, if other frames
    are used , please changle the frame selectioin in the launch file */ 
    //--------Filter Methods-----------//
    void CalculateVelocityFromPose();// calculate velocity info from pose update measurements
    void MovingWindowAveraging();// a filter using moving window
    void PushRawVelocity(Vector3d& new_linear_velocity, Vector3d& new_angular_velocity);// push newly measured velocity into raw velocity buffer
    void PushPose();//push newly measured pose into dronepose buffer
    void SetZeroVelocity();
    //--------Update Rigid-body State ------//
    rigidbody_state state;
public:
    OptiTrackFeedBackRigidBody(const char* name,ros::NodeHandle& n, unsigned int linear_window, unsigned int angular_window);
    ~OptiTrackFeedBackRigidBody();
    int GetOptiTrackState();
    void GetState(rigidbody_state& state);
    void GetRaWVelocity(Vector3d& linear_velocity,Vector3d& angular_velocity);
    void RosWhileLoopRun();// This function should be put into ros while loop
    void GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3]);
    void GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3]);
    void Veemap(Matrix3d& cross_matrix, Vector3d& vector);
    void Hatmap(Vector3d& vector, Matrix3d& cross_matrix);
};


class command_to_mavros
{
    public:
    //constructed function
    command_to_mavros(void):
        command_nh("~")
    {
        pos_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        vel_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        accel_drone_fcu_target  = Eigen::Vector3d(0.0,0.0,0.0);
        q_fcu_target            = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        euler_fcu_target        = Eigen::Vector3d(0.0,0.0,0.0);
        rates_fcu_target        = Eigen::Vector3d(0.0,0.0,0.0);
        Thrust_target           = 0.0;

        // 【订阅】无人机期望位置/速度/加速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为POSITION_TARGET_LOCAL_NED, 对应的飞控中的uORB消息为vehicle_local_position_setpoint.msg
        position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb,this);

        // 【订阅】无人机期望角度/角速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp读取), 对应Mavlink消息为ATTITUDE_TARGET (#83), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg
        attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb,this);

        // 【订阅】无人机底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力
        //  本话题来自飞控(通过Mavros功能包 /plugins/actuator_control.cpp读取), 对应Mavlink消息为ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_target_sub = command_nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 10, &command_to_mavros::actuator_target_cb,this);

        // 【发布】位置/速度/加速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
        setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // 【发布】角度/角速度期望值 坐标系 ENU系
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
        setpoint_raw_attitude_pub = command_nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

        // 【发布】底层控制量（Mx My Mz 及 F） [0][1][2][3]分别对应 roll pitch yaw控制量 及 油门推力 注意 这里是NED系的！！
        //  本话题要发送至飞控(通过Mavros功能包 /plugins/actuator_control.cpp发送), 对应Mavlink消息为SET_ACTUATOR_CONTROL_TARGET, 对应的飞控中的uORB消息为actuator_controls.msg
        actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

        // 【服务】解锁/上锁
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // 【服务】修改系统模式
        //  本服务通过Mavros功能包 /plugins/command.cpp 实现
        set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    // 相应的命令分别为 待机,起飞，移动(惯性系ENU)，移动(机体系)，悬停，降落，上锁，紧急降落
    enum Command_Type
    {
        Idle,
        Takeoff,
        Move_ENU,
        Move_Body,
        Hold,
        Land,
        Disarm,
        PPN_land,
        Trajectory_Tracking,
    };

    enum Submode_Type
    {
        XYZ_POS,
        XY_POS_Z_VEL,
        XY_VEL_Z_POS,
        XY_VEL_Z_VEL,
    };

    //Target pos of the drone [from fcu]
    Eigen::Vector3d pos_drone_fcu_target;
    //Target vel of the drone [from fcu]
    Eigen::Vector3d vel_drone_fcu_target;
    //Target accel of the drone [from fcu]
    Eigen::Vector3d accel_drone_fcu_target;
    //Target att of the drone [from fcu]
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d euler_fcu_target;
    Eigen::Vector3d rates_fcu_target;
    //Target thrust of the drone [from fcu]
    float Thrust_target;
    mavros_msgs::ActuatorControl actuator_target;



    //变量声明 - 服务
    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient arming_client;

    ros::ServiceClient set_mode_client;

    //Idle. Do nothing.
    void idle();

    //发送位置期望值至飞控（输入：期望xyz,期望yaw）
    void send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp);

    //发送速度期望值至飞控（输入：期望vxvyvz,期望yaw）
    void send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp);

    //发送速度期望值至飞控（机体系）（输入：期望vxvyvz,期望yaw）
    void send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp);

    //发送加速度期望值至飞控（输入：期望axayaz,期望yaw）
    //这是px4_pos_controller.cpp中目前使用的控制方式
    void send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp);

    //发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
    void send_attitude_setpoint(px4_command::AttitudeReference _AttitudeReference);

    //发送角度期望值至飞控（输入：期望角速度,期望推力）
    void send_attitude_rate_setpoint(Eigen::Vector3d attitude_rate_sp, float thrust_sp);

    //发送底层至飞控（输入：MxMyMz,期望推力）[Not recommanded. Because the high delay between the onboard computer and Pixhawk]
    void send_actuator_setpoint(Eigen::Vector4d actuator_sp);

    private:

        ros::NodeHandle command_nh;

        ros::Subscriber position_target_sub;
        ros::Subscriber attitude_target_sub;
        ros::Subscriber actuator_target_sub;

        ros::Publisher setpoint_raw_local_pub;
        ros::Publisher setpoint_raw_attitude_pub;
        ros::Publisher actuator_setpoint_pub;

        void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
        {
            pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

            vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

            accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
        }

        void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
        {
            q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to euler Angles
            euler_fcu_target = quaternion_to_euler(q_fcu_target);

            rates_fcu_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);

            Thrust_target = msg->thrust;
        }

        void actuator_target_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg)
        {
            actuator_target = *msg;
        }


};


OptiTrackFeedBackRigidBody::OptiTrackFeedBackRigidBody(const char* name,ros::NodeHandle& n,unsigned int linear_window, unsigned int angular_window)
{
    // load filter window size
    linear_velocity_window = linear_window;
    angular_velocity_window = angular_window;
    if(linear_velocity_window>max_windowsize)
    {
        ROS_INFO("Linear Velocity Window Size Overlimit, Max Value is [%d]",max_windowsize);
        ROS_INFO("Input Valude is [%d]",linear_velocity_window);
        linear_velocity_window = max_windowsize;
    }
    if(angular_velocity_window>max_windowsize)
    {
        ROS_INFO("Angular Velocity Window Size Overlimit, Max Value is [%d]",max_windowsize);
        ROS_INFO("Input Valude is [%d]",angular_velocity_window);
        angular_velocity_window = max_windowsize;
    }
    // set up subscriber to vrpn optitrack beedback
    subOptiTrack = n.subscribe(name, 1, &OptiTrackFeedBackRigidBody::OptiTrackCallback,this);
    //Initialize all velocity
    for(int i =0;i<max_windowsize;i++)
    {
        velocity_raw[i](0)=0;
        velocity_raw[i](1)=0;
        velocity_raw[i](2)=0;
        angular_velocity_raw[i](0)=0;
        angular_velocity_raw[i](1)=0;
        angular_velocity_raw[i](2)=0;
    }
    velocity_filtered(0)=0;
    velocity_filtered(1)=0;
    velocity_filtered(2)=0;
    angular_velocity_filtered(0)=0;
    angular_velocity_filtered(1)=0;
    angular_velocity_filtered(2)=0;
    //Initialize all pose
    for(int i = 0;i<2;i++)
    {
        pose[i].q0 = 1;
        pose[i].q1 = 0;
        pose[i].q2 = 0;
        pose[i].q3 = 0;
        pose[i].t = 0;
        pose[i].Position(0) = 0;
        pose[i].Position(1) = 0;
        pose[i].Position(2) = 0;
        pose[i].L<< 0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
        pose[i].R<< 0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;   
        pose[1].R_IB<< 1,0,0,
                    0,1,0,
                    0,0,1;
        pose[1].R_BI<< 1,0,0,
                    0,1,0,
                    0,0,1;                    
    }
    // Initialize flag
    OptiTrackFlag = 0;
    FeedbackState = 0;
}

void OptiTrackFeedBackRigidBody::CalculateVelocityFromPose()
{

    /* Logic:
     * 1) push the current pose into buffer
     * 2) determine whether the buffer has a valid time value  (pose[0].t >0); if so calculate velocity
     * 3) if not just set the velocity_onestep as zero
     * 4) push current time, and velocity_onestep into the velocity buffer
     * 5) update the filtered velocity
    */
    // perform the Logic:
    // step (1): push the current pose into buffer
    PushPose();
    // step (2): determine whether the buffer has a valid time value  (pose[0].t >0); if so calculate velocity
    double dt = 0.0;
    Vector3d velocity_onestep;
    Vector3d angular_velocity_onestep;
  if (pose[0].t >0)// calculate only when last time stamp has been recorded.
  {
      // step (2)
      dt = pose[1].t - pose[0].t;// time step
      // calculate linear velocity
      velocity_onestep = (pose[1].Position- pose[0].Position)/dt;
      // calculate angular velocity
      Matrix3d RotationDifference = - pose[1].R_BI*pose[0].R_IB/dt;
      Veemap(RotationDifference,angular_velocity_onestep);
  }else// step (3): if not set velocity to zero
  {
      velocity_onestep(0) = 0.0;
      velocity_onestep(1) = 0.0;
      velocity_onestep(2) = 0.0;
      angular_velocity_onestep(0) = 0.0;
      angular_velocity_onestep(1) = 0.0;
      angular_velocity_onestep(2) = 0.0;
  }
  // step (4): push current time, and velocity_onestep into the velocity buffer
  PushRawVelocity(velocity_onestep,angular_velocity_onestep);
  // step (5): update filtered velocity
  MovingWindowAveraging();
}
void OptiTrackFeedBackRigidBody::PushPose()
{
    pose[0] = pose[1];// straightforward push the pose into buffer
    // update the latest pose
    double t_current = (double)OptiTrackdata.header.stamp.sec + (double)OptiTrackdata.header.stamp.nsec*0.000000001;
    pose[1].t = t_current;
    // take a special note at the order of the quaterion
    pose[1].q0 = OptiTrackdata.pose.orientation.w;
    pose[1].q1 = OptiTrackdata.pose.orientation.x;
    pose[1].q2 = OptiTrackdata.pose.orientation.y;
    pose[1].q3 = OptiTrackdata.pose.orientation.z;
    // update the auxiliary matrix
    /*
    L = [-q1 q0 q3 -q2;
         -q2 -q3 q0 q1;
         -q3 q2 -q1 q0]
    R = [-q1 q0 -q3 q2;
         -q2 q3 q0 -q1;
         -q3 -q2 q1 q0]
    R_IB = RL^T
    */
    pose[1].L(0,0) = - pose[1].q1;
    pose[1].L(1,0) = - pose[1].q2;
    pose[1].L(2,0) = - pose[1].q3;

    pose[1].L(0,1) = pose[1].q0;
    pose[1].L(1,2) = pose[1].q0;
    pose[1].L(2,3) = pose[1].q0;

    pose[1].L(0,2) = pose[1].q3;
    pose[1].L(0,3) = - pose[1].q2;
    pose[1].L(1,1) = - pose[1].q3;
    pose[1].L(1,3) = pose[1].q1;
    pose[1].L(2,1) = pose[1].q2;
    pose[1].L(2,2) = - pose[1].q1;

    pose[1].R(0,0) = - pose[1].q1;
    pose[1].R(1,0) = - pose[1].q2;
    pose[1].R(2,0) = - pose[1].q3;

    pose[1].R(0,1) = pose[1].q0;
    pose[1].R(1,2) = pose[1].q0;
    pose[1].R(2,3) = pose[1].q0;

    pose[1].R(0,2) = -pose[1].q3;
    pose[1].R(0,3) =  pose[1].q2;
    pose[1].R(1,1) =  pose[1].q3;
    pose[1].R(1,3) = -pose[1].q1;
    pose[1].R(2,1) = -pose[1].q2;
    pose[1].R(2,2) =  pose[1].q1; 

    pose[1].R_IB = pose[1].R * pose[1].L.transpose();
    pose[1].R_BI = pose[1].R_IB.transpose();
    // position is straight forward
    pose[1].Position(0) =  OptiTrackdata.pose.position.x;
    pose[1].Position(1) =  OptiTrackdata.pose.position.y;
    pose[1].Position(2) =  OptiTrackdata.pose.position.z;
}

void OptiTrackFeedBackRigidBody::PushRawVelocity(Vector3d& new_linear_velocity, Vector3d& new_angular_velocity)
{
    /* Logic:
     * a(i-1) = a(i), i = 2...windowsize
     * should fristly start from  i = 2. a(1) = a(2); a(2) = a(3);....; a(N-1) = a(N)
     * secondly a(N) = a_new
    */
   // linear velocity
    for(int i = 1;i<linear_velocity_window;i++)//first step
    {
        velocity_raw[i-1] = velocity_raw[i];
    }
    velocity_raw[linear_velocity_window-1] = new_linear_velocity;// second step update the last variable in the velocity buffer
    // angular velocity
    for(int i = 1;i<angular_velocity_window;i++)//first step
    {
        angular_velocity_raw[i-1] = angular_velocity_raw[i];
    }
    angular_velocity_raw[angular_velocity_window-1] = new_angular_velocity;// second step update the last variable in the velocity buffer   
}

void OptiTrackFeedBackRigidBody::MovingWindowAveraging()
{

    /* Logic: Average the raw velocity measurement in the
    */
    double weight_linear = (double)1/linear_velocity_window;// the weight on each velocity to be summed up.
    double weight_angular = (double)1/angular_velocity_window;// the weight on each velocity to be summed up.
    // create a temporary variable to store the summed velocity and initialize it witht the 1st buffer value
    Vector3d velocitytemp;
    Vector3d angular_velocitytemp;
    velocitytemp = weight_linear*velocity_raw[0];
    angular_velocitytemp = weight_angular*angular_velocity_raw[0];

    for(int i = 1;i<linear_velocity_window;i++)// sum starts from the second buffer value
    {
        velocitytemp += weight_linear*velocity_raw[i];
    }
    for(int i = 1;i<angular_velocity_window;i++)// sum starts from the second buffer value
    {
        angular_velocitytemp += weight_angular*angular_velocity_raw[i];
    }
    // the filtered vlocity is just the weighted summed result
    velocity_filtered = velocitytemp;
    angular_velocity_filtered = angular_velocitytemp;
}

void OptiTrackFeedBackRigidBody::GetState(rigidbody_state& state)
{
    state.time_stamp = pose[1].t;
    state.Position = pose[1].Position;
    state.V_I = velocity_filtered;
    state.Omega_BI = angular_velocity_filtered;
    Hatmap(state.Omega_BI,state.Omega_Cross);
    state.R_IB = pose[1].R_IB;
    state.R_BI = pose[1].R_BI; 
    double euler_temp[3];
    GetEulerAngleFromQuaterion_NormalConvention(euler_temp);
    state.Euler(0) = euler_temp[0];// euler angle
    state.Euler(1) = euler_temp[1];// euler angle
    state.Euler(2) = euler_temp[2];// euler angle
    state.quaterion(0) = pose[1].q0;
    state.quaterion(1) = pose[1].q1;
    state.quaterion(2) = pose[1].q2;
    state.quaterion(3) = pose[1].q3;
}
void OptiTrackFeedBackRigidBody::GetRaWVelocity(Vector3d& linear_velocity,Vector3d& angular_velocity)
{
    linear_velocity = velocity_raw[linear_velocity_window-1];// return the filtered velocity
    angular_velocity = angular_velocity_raw[angular_velocity_window-1];// return the filtered velocity
}
void  OptiTrackFeedBackRigidBody::SetZeroVelocity()
{
    for(int i =0;i<linear_velocity_window;i++)
    {
        velocity_raw[i](0)=0;
        velocity_raw[i](1)=0;
        velocity_raw[i](2)=0;
    }
    for(int i =0;i<angular_velocity_window;i++)
    {
        angular_velocity_raw[i](0)=0;
        angular_velocity_raw[i](1)=0;
        angular_velocity_raw[i](2)=0;
    }
    velocity_filtered(0)=0;
    velocity_filtered(1)=0;
    velocity_filtered(2)=0;
    angular_velocity_filtered(0) =0;
    angular_velocity_filtered(1) =0;
    angular_velocity_filtered(2) =0;
}

void OptiTrackFeedBackRigidBody::RosWhileLoopRun()
{
    if(OptiTrackFlag==1)
    {// update the velocity only when there is OptiTrack feedback
        CalculateVelocityFromPose();
        FeedbackState=1;
    }else{
        // if the optitrack measurements no longer feedback, when the pose update will stop and we only return 0 velocity
        SetZeroVelocity();
        FeedbackState=0;
    }

    OptiTrackFlag = 0;// reset the feedback flag to 0
}
int OptiTrackFeedBackRigidBody::GetOptiTrackState()
{
    if (FeedbackState==1) {
      ROS_INFO("OptiTrack:Normal");
    }else{
      ROS_INFO("OptiTrack:No FeedBack");
    }
    ROS_INFO("Linear Velocity Filter Window Size is [%d]",linear_velocity_window);
    ROS_INFO("Angular Velocity Filter Window Size is [%d]",angular_velocity_window);
    return FeedbackState;
}
void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_NormalConvention(double (&eulerangle)[3])
{


    /* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    */
//    eulerangle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
//    eulerangle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
//    eulerangle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));


    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q2 * pose[1].q3);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q2 * pose[1].q2);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q2 - pose[1].q3 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q3 + pose[1].q1 * pose[1].q2);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    //double yaw  = atan2(2.0 * (dronepose[1].q3 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q2), -1.0 + 2.0 * (dronepose[1].q0 * dronepose[1].q0 + dronepose[1].q1 * dronepose[1].q1));
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::GetEulerAngleFromQuaterion_OptiTrackYUpConvention(double (&eulerangle)[3])
{

    // OptiTrack gives a quaternion with q2 and q3 flipped. (and sign flipped for q3)
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (pose[1].q0 * pose[1].q1 + pose[1].q3 * pose[1].q2);
    double cosr_cosp = +1.0 - 2.0 * (pose[1].q1 * pose[1].q1 +pose[1].q3 * pose[1].q3);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (pose[1].q0 * pose[1].q3 - pose[1].q2 * pose[1].q1);
    double pitch;
    if (fabs(sinp) >= 1)
           pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
           pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (pose[1].q0 * pose[1].q2 + pose[1].q1 * pose[1].q3);
    double cosy_cosp = +1.0 - 2.0 * (pose[1].q2 * pose[1].q2 + pose[1].q3 * pose[1].q3);
    double yaw = atan2(siny_cosp, cosy_cosp);
    eulerangle[0] = roll;
    eulerangle[1] = pitch;
    eulerangle[2] = yaw;

}

void OptiTrackFeedBackRigidBody::OptiTrackCallback(const geometry_msgs::PoseStamped& msg)
{
        // must use head information to distiguish the correct 
        OptiTrackdata = msg; // update optitrack data
        OptiTrackFlag = 1;// signal a new measurement feed has been revcieved.
}

OptiTrackFeedBackRigidBody::~OptiTrackFeedBackRigidBody()
{

}

void OptiTrackFeedBackRigidBody::Veemap(Matrix3d& cross_matrix, Vector3d& vector)
{
    vector(0) = -cross_matrix(1,2);
    vector(1) = cross_matrix(0,2);
    vector(2) = -cross_matrix(0,1);
}
void OptiTrackFeedBackRigidBody::Hatmap(Vector3d& vector, Matrix3d& cross_matrix)
{
    /*
    r^x = [0 -r3 r2;
           r3 0 -r1;
          -r2 r1 0]
    */
    
    cross_matrix(0,0) = 0.0;
    cross_matrix(0,1) = - vector(2);
    cross_matrix(0,2) = vector(1);

    cross_matrix(1,0) = vector(2);
    cross_matrix(1,1) = 0.0;
    cross_matrix(1,2) = - vector(0);

    cross_matrix(2,0) = - vector(1);
    cross_matrix(2,1) = vector(0);
    cross_matrix(2,2) = 0.0;

}

#endif


