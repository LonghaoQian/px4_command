/***************************************************************************************************************************
 * px4_pos_estimator.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.3.10
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息,从laser坐标系转换至NED坐标系
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/
/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  PX4 Position Estimator using external positioning equipment
*         1. Subscribe position and yaw information from Lidar SLAM node(cartorgrapher_ros节点), transfrom from laser frame to ENU frame
*         2. Subscribe position and yaw information from Vicon node(vrpn-client-ros节点), transfrom from vicon frame to ENU frame
*         3. Send the position and yaw information to FCU using Mavros package (/mavros/mocap/pose or /mavros/vision_estimate/pose)
*         4. Subscribe position and yaw information from FCU, used for compare
***************************************************************************************************************************/


//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>


#include <math_utils.h>
#include <Frame_tf_utils.h>
//msg 头文件
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
#include <sensor_msgs/Range.h>


using namespace std;
//---------------------------------------相关参数-----------------------------------------------
int flag_use_laser_or_vicon;                               //0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap;                          //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap;                              //无人机当前姿态 (vicon)
//---------------------------------------laser定位相关------------------------------------------
Eigen::Vector3d pos_drone_laser;                          //无人机当前位置 (laser)
Eigen::Quaterniond q_laser;
Eigen::Vector3d Euler_laser;                                         //无人机当前姿态(laser)

geometry_msgs::TransformStamped laser;                          //当前时刻cartorgrapher发布的数据
geometry_msgs::TransformStamped laser_last;
//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;                           //无人机当前位置 (来自fcu)
Eigen::Vector3d vel_drone_fcu;                           //无人机上一时刻位置 (来自fcu)

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;                                          //无人机当前欧拉角(来自fcu)
//---------------------------------------发布相关变量--------------------------------------------
geometry_msgs::PoseStamped vision;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_dt(ros::Time last);                                                        //获取时间间隔
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void laser_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    //确定是cartographer发出来的/tf信息
    //有的时候/tf这个消息的发布者不止一个
    if (msg->transforms[0].header.frame_id == "map")
    {
        laser = msg->transforms[0];

        float dt_laser;

        dt_laser = (laser.header.stamp.sec - laser_last.header.stamp.sec) + (laser.header.stamp.nsec - laser_last.header.stamp.nsec)/10e9;

        //这里需要做这个判断是因为cartographer发布位置时有一个小bug，ENU到NED不展开讲。
        if (dt_laser != 0)
        {
            //位置 xy  [将解算的位置从laser坐标系转换至ENU坐标系]???
            pos_drone_laser[0]  = laser.transform.translation.x;
            pos_drone_laser[1]  = laser.transform.translation.y;

            // Read the Quaternion from the Carto Package [Frame: Laser[ENU]]
            Eigen::Quaterniond q_laser_enu(laser.transform.rotation.w, laser.transform.rotation.x, laser.transform.rotation.y, laser.transform.rotation.z);

            q_laser = q_laser_enu;

            // Transform the Quaternion to Euler Angles
            Euler_laser = quaternion_to_euler(q_laser);
        }

        laser_last = laser;
    }
}
void sonic_cb(const std_msgs::UInt16::ConstPtr& msg)
{
    std_msgs::UInt16 sonic;

    sonic = *msg;

    //位置
    pos_drone_laser[2]  = (float)sonic.data / 1000;
}

void tfmini_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    sensor_msgs::Range tfmini;

    tfmini = *msg;

    //位置
    pos_drone_laser[2]  = tfmini.range ;

}

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 ENU系
    int optitrack_frame = 0; //Frame convention 0: Z-up -- 1: Y-up

    if(optitrack_frame == 0)
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        pos_drone_mocap = Eigen::Vector3d(-msg->pose.position.x,msg->pose.position.z,msg->pose.position.y);
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    q_fcu = q_fcu_enu;

    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);

    // Transform the Quaternion from ENU to NED
    Eigen::Quaterniond q_ned = transform_orientation_enu_to_ned( transform_orientation_baselink_to_aircraft(q_fcu_enu) );
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    //读取参数表中的参数
    // 使用激光SLAM数据orVicon数据 0 for vicon， 1 for 激光SLAM
    nh.param<int>("pos_estimator/flag_use_laser_or_vicon", flag_use_laser_or_vicon, 0);

    // 【订阅】cartographer估计位置
    ros::Subscriber laser_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1000, laser_cb);

    // 【订阅】超声波的数据
    ros::Subscriber sonic_sub = nh.subscribe<std_msgs::UInt16>("/sonic", 100, sonic_cb);

    // 【订阅】tf mini的数据
    ros::Subscriber tfmini_sub = nh.subscribe<sensor_msgs::Range>("/TFmini/TFmini", 100, tfmini_cb);

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV/pose", 1000, optitrack_cb);

    // 【订阅】无人机当前位置 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系 这里订阅仅作比较用
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#??), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);

    // 频率
    ros::Rate rate(20.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //vicon
        if(flag_use_laser_or_vicon == 0)
        {
            vision.pose.position.x = pos_drone_mocap[0] ;
            vision.pose.position.y = pos_drone_mocap[1] ;
            vision.pose.position.z = pos_drone_mocap[2] ;

            vision.pose.orientation.x = q_mocap.x();
            vision.pose.orientation.y = q_mocap.y();
            vision.pose.orientation.z = q_mocap.z();
            vision.pose.orientation.w = q_mocap.w();

        }//laser
        else if (flag_use_laser_or_vicon == 1)
        {
            vision.pose.position.x = pos_drone_laser[0];
            vision.pose.position.y = pos_drone_laser[1];
            vision.pose.position.z = pos_drone_laser[2];

            vision.pose.orientation.x = q_laser.x();
            vision.pose.orientation.y = q_laser.y();
            vision.pose.orientation.z = q_laser.z();
            vision.pose.orientation.w = q_laser.w();
        }

        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);

        //打印
        printf_info();
        rate.sleep();
    }

    return 0;

}

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    //using vicon system
    if(flag_use_laser_or_vicon == 0)
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>Vicon Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_vicon [X Y Z] : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
        cout << "Euler_vicon [Yaw] : " << Euler_mocap[2] * 180/M_PI<<" [deg]  "<<endl;
    }else
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>Laser Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_laser [X Y Z] : " << pos_drone_laser[0] << " [ m ] "<< pos_drone_laser[1] <<" [ m ] "<< pos_drone_laser[2] <<" [ m ] "<<endl;
        cout << "Euler_vlaser[Yaw] : " << Euler_laser[2] * 180/M_PI<<" [deg]  "<<endl;
    }

        cout <<">>>>>>>>>>>>>>>>>>>>>>>>FCU Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1] <<" [ m ] "<< pos_drone_fcu[2] <<" [ m ] "<<endl;
        cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1] <<" [m/s] "<< vel_drone_fcu[2] <<" [m/s] "<<endl;
        cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;
}
