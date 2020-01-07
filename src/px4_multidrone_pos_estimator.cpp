/***************************************************************************************************************************
 * px4_multidrone_pos_estimator.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2019.8.31
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息,从laser坐标系转换至NED坐标系
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/
//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <state_from_mavros_multidrone.h>
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
#include <px4_command/DroneState.h>
#include <px4_command/Mocap.h>
using namespace std;
struct SubTopic
{
    char str[100];
};
struct PubTopic
{
    char str[100];
};
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
Eigen::Vector3d Att_fcu;                               //无人机当前欧拉角(来自fcu)
Eigen::Vector3d Att_rate_fcu;
//---------------------------------------发布相关变量--------------------------------------------
ros::Publisher vision_pub;
ros::Publisher drone_state_pub;
px4_command::DroneState _DroneState;  
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> callbacks <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
    //Frame convention 0: Z-up -- 1: Y-up (See the configuration in the motive software)
    int optitrack_frame = 0; 
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

void send_to_fcu()
{
    geometry_msgs::PoseStamped vision;
    
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
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_multidrone_pos_estimator");
    ros::NodeHandle nh("~");
    // 频率
    ros::Rate rate(50.0);
    //读取参数表中的参数
    // 使用激光SLAM数据orVicon数据 0 for vicon， 1 for 激光SLAM
    nh.param<int>("pos_estimator/flag_use_laser_or_vicon", flag_use_laser_or_vicon, 0);
    /*----------- determine the  ID of the drone -----------------------------*/
    SubTopic vrpn_client_node_UAV_pose;
    SubTopic tf;
    SubTopic sonic;
    SubTopic TFmini_TFmini;
    PubTopic mavros_vision_pose_pose;
    PubTopic px4_command_drone_state;
    // add preflex: (mavros and px4_command use lower case uav, while /vrpn_client_node/UAV use upper case UAV)
    strcpy (vrpn_client_node_UAV_pose.str,"/vrpn_client_node/UAV"); 
    strcpy (tf.str,"/uav"); 
    strcpy (sonic.str,"/uav"); 
    strcpy (TFmini_TFmini.str,"/uav"); 
    strcpy (mavros_vision_pose_pose.str,"/uav"); 
    strcpy (px4_command_drone_state.str,"/uav");
 
    char ID[20]; // ID of the uav

    if ( argc > 1) {
    // if ID is specified as the second argument 
        strcat (vrpn_client_node_UAV_pose.str,argv[1]);
        strcat (tf.str,argv[1]);
        strcat (sonic.str,argv[1]);
        strcat (TFmini_TFmini.str,argv[1]); 
        strcat (mavros_vision_pose_pose.str,argv[1]);
        strcat (px4_command_drone_state.str,argv[1]);
        ROS_INFO("UAV ID specified as: UAV%s", argv[1]);
        strcpy (ID,argv[1]);
    } else {
        // if ID is not specified, then set the drone to UAV0
        strcat (vrpn_client_node_UAV_pose.str,"0");
        strcat (tf.str,"0");
        strcat (sonic.str,"0");
        strcat (TFmini_TFmini.str,"0"); 
        strcat (mavros_vision_pose_pose.str,"0");
        strcat (px4_command_drone_state.str,"0");
        ROS_WARN("NO UAV ID specified, set ID to 0.");
        strcpy (ID,"0");
    }

    state_from_mavros_multidrone _state_from_mavros(ID); // define the state_from_mavros 
    strcat (vrpn_client_node_UAV_pose.str,"/pose");
    strcat (tf.str,"/tf");
    strcat (sonic.str,"/sonic");
    strcat (TFmini_TFmini.str,"/TFmini/TFmini"); 
    strcat (mavros_vision_pose_pose.str,"/mavros/vision_pose/pose");
    strcat (px4_command_drone_state.str,"/px4_command/drone_state");

    ROS_INFO("Subscribe uav Mocap from: %s", vrpn_client_node_UAV_pose.str);
    ROS_INFO("Subscribe TFMessage from: %s", tf.str);
    ROS_INFO("Subscribe sonic from: %s", sonic.str);
    ROS_INFO("Subscribe TFmini from: %s", TFmini_TFmini.str); 
    ROS_INFO("Publish mavros_vision_pose_pose to: %s", mavros_vision_pose_pose.str);
    ROS_INFO("Publish PoseStamped to: %s", px4_command_drone_state.str);

    // 【订阅】cartographer估计位置
    ros::Subscriber laser_sub = nh.subscribe<tf2_msgs::TFMessage>(tf.str, 1000, laser_cb);

    // 【订阅】超声波的数据
    ros::Subscriber sonic_sub = nh.subscribe<std_msgs::UInt16>(sonic.str, 100, sonic_cb);

    // 【订阅】tf mini的数据
    ros::Subscriber tfmini_sub = nh.subscribe<sensor_msgs::Range>(TFmini_TFmini.str, 100, tfmini_cb);

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>(vrpn_client_node_UAV_pose.str, 1000, optitrack_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#??), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>(mavros_vision_pose_pose.str, 100);

    drone_state_pub = nh.advertise<px4_command::DroneState>(px4_command_drone_state.str, 100);

    ROS_INFO("Start the estimator...");

    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        // 将定位信息及偏航角信息发送至飞控，根据参数flag_use_laser_or_vicon选择定位信息来源
        send_to_fcu();

        for (int i=0;i<3;i++)
        {
            pos_drone_fcu[i] = _state_from_mavros._DroneState.position[i];                           
            vel_drone_fcu[i] = _state_from_mavros._DroneState.velocity[i];                           
            Att_fcu[i] = _state_from_mavros._DroneState.attitude[i];  
            Att_rate_fcu[i] = _state_from_mavros._DroneState.attitude_rate[i];  
        }

        // 发布无人机状态至px4_pos_controller.cpp节点，根据参数Use_mocap_raw选择位置速度消息来源
        // get drone state from _state_from_mavros
        _DroneState = _state_from_mavros._DroneState;
        _DroneState.header.stamp = ros::Time::now();
        drone_state_pub.publish(_DroneState);
        /*TODO add a vrpn detector*/
        rate.sleep();
    }

    return 0;

}