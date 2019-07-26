//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>


#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <px4_command_utils.h>
#include <OptiTrackFeedBackRigidBody.h>
#include <px4_command/ControlCommand.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/MocapInfo.h>

#include <px4_command/Trajectory.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>

#include <bitset>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <px4_command/Topic_for_log.h>


using namespace std;
//---------------------------------------相关参数-----------------------------------------------
px4_command::Topic_for_log _Topic_for_log;
px4_command::MocapInfo UAV_motion;
px4_command::MocapInfo Payload_motion;
Eigen::Vector3d pos_drone_mocap;                          //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap;                              //无人机当前姿态 (vicon)
rigidbody_state UAVstate;
rigidbody_state Payloadstate;
Eigen::Quaterniond q_fcu_target;
Eigen::Vector3d euler_fcu_target;
float Thrust_target;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void printf_info();                                                                       //打印函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void log_cb(const px4_command::Topic_for_log::ConstPtr &msg)
{
    _Topic_for_log = *msg;
}

void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    //Transform the Quaternion to euler Angles
    euler_fcu_target = quaternion_to_euler(q_fcu_target);

    Thrust_target = msg->thrust;
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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_station");
    ros::NodeHandle nh("~");

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV/pose", 10, optitrack_cb);

    ros::Subscriber log_sub = nh.subscribe<px4_command::Topic_for_log>("/px4_command/topic_for_log", 10, log_cb);

    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10,att_target_cb);

    // publish 
    ros::Publisher motion_pub = nh.advertise<px4_command::MocapInfo>("/px4_command/mocapinfo", 10);

    // 频率
    ros::Rate rate(100.0);

    OptiTrackFeedBackRigidBody UAV("/vrpn_client_node/UAV/pose",nh,3,3);
    OptiTrackFeedBackRigidBody Payload("/vrpn_client_node/Payload/pose",nh,3,3);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //利用OptiTrackFeedBackRigidBody类获取optitrack的数据
        //UAV.GetOptiTrackState();

        UAV.RosWhileLoopRun();
        UAV.GetState(UAVstate);
        Payload.RosWhileLoopRun();
        Payload.GetState(Payloadstate);
        UAV_motion.header.stamp = ros::Time::now();

        for(int i = 0;i<3;i++)
        {
            UAV_motion.position[i] = UAVstate.Position(i);
            UAV_motion.velocity[i] = UAVstate.V_I(i);
            UAV_motion.angular_velocity[i] =  UAVstate.Omega_BI(i);
            UAV_motion.Euler[i] = UAVstate.Euler(i);

            Payload_motion.position[i] = Payloadstate.Position(i);
            Payload_motion.velocity[i] = Payloadstate.V_I(i);
            Payload_motion.angular_velocity[i] =  Payloadstate.Omega_BI(i);
            Payload_motion.Euler[i] = Payloadstate.Euler(i);

        }



        motion_pub.publish(UAV_motion);

        //打印
        printf_info();
        rate.sleep();
    }

    return 0;

}

void printf_info()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Ground Station  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
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

    px4_command_utils::prinft_drone_state(_Topic_for_log.Drone_State);

    px4_command_utils::printf_command_control(_Topic_for_log.Control_Command);

    px4_command_utils::prinft_attitude_reference(_Topic_for_log.Attitude_Reference);


    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Control Output  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    cout << "u_l [X Y Z]  : " << _Topic_for_log.Control_Output.u_l[0] << " [ ] "<< _Topic_for_log.Control_Output.u_l[1] <<" [ ] "<< _Topic_for_log.Control_Output.u_l[2] <<" [ ] "<<endl;
    
    cout << "u_d [X Y Z]  : " << _Topic_for_log.Control_Output.u_d[0] << " [ ] "<< _Topic_for_log.Control_Output.u_d[1] <<" [ ] "<< _Topic_for_log.Control_Output.u_d[2] <<" [ ] "<<endl;
    cout << "NE  [X Y Z]  : " << _Topic_for_log.Control_Output.NE[0] << " [ ] "<< _Topic_for_log.Control_Output.NE[1] <<" [ ] "<< _Topic_for_log.Control_Output.NE[2] <<" [ ] "<<endl;

    cout << "Thrust  [X Y Z]  : " << _Topic_for_log.Control_Output.Thrust[0] << " [ ] "<< _Topic_for_log.Control_Output.Thrust[1] <<" [ ] "<< _Topic_for_log.Control_Output.Thrust[2] <<" [ ] "<<endl;

    cout << "Throttle  [X Y Z]  : " << _Topic_for_log.Control_Output.Throttle[0] << " [ ] "<< _Topic_for_log.Control_Output.Throttle[1] <<" [ ] "<< _Topic_for_log.Control_Output.Throttle[2] <<" [ ] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>> Target Info FCU <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    
    cout << "Pos_vicon [X Y Z]  : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
    
    cout << "Att_target [R P Y] : " << euler_fcu_target[0] * 180/M_PI <<" [deg]  "<<euler_fcu_target[1] * 180/M_PI << " [deg]  "<< euler_fcu_target[2] * 180/M_PI<<" [deg]  "<<endl;
    
    cout << "Thr_target [ 0-1 ] : " << Thrust_target <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>Error Info [ Longhao ]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Error_pos      : " << UAVstate.Position[0] - _Topic_for_log.Drone_State.position[0] << " [ m ] "<< UAVstate.Position[1] - _Topic_for_log.Drone_State.position[1]<<" [ m ] "<< UAVstate.Position[2] - _Topic_for_log.Drone_State.position[2]<<" [ m ] "<<endl;
    cout << "Error_vel      : " << UAVstate.V_I[0] - _Topic_for_log.Drone_State.velocity[0] << " [m/s] "<< UAVstate.V_I[1] - _Topic_for_log.Drone_State.velocity[1]<<" [m/s] "<< UAVstate.V_I[2] - _Topic_for_log.Drone_State.velocity[2]<<" [m/s] "<<endl;
    cout << "Error_att      : " << UAVstate.Euler[0]*57.3 - _Topic_for_log.Drone_State.attitude[0]*57.3 << " [deg] "<< UAVstate.Euler[1]*57.3 - _Topic_for_log.Drone_State.attitude[1]*57.3<<" [deg] "<< UAVstate.Euler[2]*57.3 - _Topic_for_log.Drone_State.attitude[2]*57.3<<" [deg] "<<endl;
    cout << "Error_att_rate : " << UAVstate.Omega_BI[0]*57.3 - _Topic_for_log.Drone_State.attitude_rate[0]*57.3 << " [deg] "<< UAVstate.Omega_BI[1]*57.3 - _Topic_for_log.Drone_State.attitude_rate[1]*57.3<<" [deg] "<< UAVstate.Omega_BI[2]*57.3 - _Topic_for_log.Drone_State.attitude_rate[2]*57.3<<" [deg] "<<endl;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>Payload Info [ Longhao ]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Payload Relative Position: " << _Topic_for_log.Drone_State.payload_pos[0] - _Topic_for_log.Drone_State.position[0] << " [ m ] "<<  _Topic_for_log.Drone_State.payload_pos[1] - _Topic_for_log.Drone_State.position[1] <<" [ m ] "<<  _Topic_for_log.Drone_State.payload_pos[2] - _Topic_for_log.Drone_State.position[2] <<" [ m ] "<<endl;
    cout << "Payload Relative Velocity: " << _Topic_for_log.Drone_State.payload_vel[0] - _Topic_for_log.Drone_State.velocity[0] << " [ m ] "<< _Topic_for_log.Drone_State.payload_vel[1] - _Topic_for_log.Drone_State.velocity[1]<<" [ m ] "<< _Topic_for_log.Drone_State.payload_vel[2] - _Topic_for_log.Drone_State.velocity[2]<<" [ m ] "<<endl;
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>Mocap_UAV_Topic<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    cout << "Mocap_UAV_Position"<<UAV_motion.position[0]<<"m"<<UAV_motion.position[1]<<"m"<<UAV_motion.position[2]<<"m"<<endl;
    cout << "Mocap_UAV_velocity"<<UAV_motion.velocity[0]<<"m/s"<<UAV_motion.velocity[1]<<"m/s"<<UAV_motion.velocity[2]<<"m/s"<<endl;
    cout << "Mocap_UAV_angular_velocity"<<UAV_motion.angular_velocity[0]<<"rad/s"<<UAV_motion.angular_velocity[1]<<"rad/s"<<UAV_motion.angular_velocity[2]<<"rad/s"<<endl;
    cout << "Mocap_UAV_Euler"<<UAV_motion.Euler[0]<<"rad"<<UAV_motion.Euler[1]<<"rad"<<UAV_motion.Euler[2]<<"rad"<<endl;
}
