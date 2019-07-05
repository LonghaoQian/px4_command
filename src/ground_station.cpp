

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
px4_command::DroneState _DroneState;                         //无人机状态量


px4_command::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d euler_fcu_target;
    float Thrust_target;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    _DroneState.position[0] = msg->pose.position.x;
    _DroneState.position[1] = msg->pose.position.y;
    _DroneState.position[2] = msg->pose.position.z;
}

void output_cb(const px4_command::AttitudeReference::ConstPtr &msg)
{
    _AttitudeReference = *msg;
}

void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

//Transform the Quaternion to euler Angles
euler_fcu_target = quaternion_to_euler(q_fcu_target);

Thrust_target = msg->thrust;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
    //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);


    ros::Subscriber command_sub = nh.subscribe<px4_command::AttitudeReference>("/px4_command/output", 10,output_cb);


    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10,att_target_cb);

    // 频率
    ros::Rate rate(10.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //打印
        printf_info();
        rate.sleep();
    }

    return 0;

}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>Ground Station<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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

        cout << "Position [X Y Z] : " << _DroneState.position[0] << " [ m ] "<< _DroneState.position[1]<<" [ m ] "<<_DroneState.position[2]<<" [ m ] "<<endl;



    cout << "desired_thrust_normalized [X Y Z] : " << _AttitudeReference.thrust_sp[0] << " [m/s^2] "<< _AttitudeReference.thrust_sp[1]<<" [Nm/s^2] "<<_AttitudeReference.thrust_sp[2]<<" [m/s^2] "<<endl;
   cout << "desired_attitude [R P Y] : " << _AttitudeReference.desired_attitude[0] * 180/M_PI <<" [deg] "<<_AttitudeReference.desired_attitude[1] * 180/M_PI << " [deg] "<< _AttitudeReference.desired_attitude[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "desired_throttle [0-1] : " << _AttitudeReference.desired_throttle <<endl;

    cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg] "<<euler_fcu_target[1] * 180/M_PI << " [deg] "<< euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;
    cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;


}
