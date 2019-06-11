/***************************************************************************************************************************
* px4_sender.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  PX4 command sender using px4 default command
*         1. Subscribe command.msg from upper nodes
*         2. Send command using command_to_mavros.h
*         3. Command includes:  (1)xyz+yaw
*                               (2)takeoff
*                               (3)land
*                               (4)idle
*                               (5)loiter
*                               (6)xyz+yaw(body frame)
***************************************************************************************************************************/

#include <ros/ros.h>

#include <command_to_mavros.h>
#include <px4_command/command.h>

#include <Eigen/Eigen>

using namespace std;
using namespace namespace_command_to_mavros;

//自定义的Command变量
//相应的命令分别为 待机,起飞，移动(惯性系ENU)，移动(机体系)，悬停，降落，上锁，紧急降落
enum Command
{
    Idle,
    Takeoff,
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    Failsafe_land,
};

float Takeoff_height;
float Disarm_height;

//Command Now [from upper node]
px4_command::command Command_Now;                      //无人机当前执行命令

//Command Last [from upper node]
px4_command::command Command_Last;                     //无人机上一条执行命令

float get_ros_time(ros::Time begin);
void prinft_command_state();

void Command_cb(const px4_command::command::ConstPtr& msg)
{
    Command_Now = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_sender");
    ros::NodeHandle nh("~");

    ros::Subscriber Command_sub = nh.subscribe<px4_command::command>("/px4/command", 10, Command_cb);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);

    cout << "Takeoff_height"<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height "<< Disarm_height <<" [m] "<<endl;

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    Eigen::Vector3d pos_sp(0,0,0);

    Eigen::Vector3d vel_sp(0,0,0);

    double yaw_sp;

    command_to_mavros pos_sender;

    pos_sender.printf_param();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 等待和飞控的连接
    while(ros::ok() && pos_sender.current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    // 连接成功
    ROS_INFO("Connected!!");

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();

    }

    Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
    //Set the takeoff position
    Takeoff_position = pos_sender.pos_drone_fcu;

    Command_Now.comid = 0;
    Command_Now.command = Idle;

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        ros::spinOnce();


        float cur_time = get_ros_time(begin_time);

        pos_sender.prinft_drone_state(cur_time);
        prinft_command_state();
        //Check for geo fence: If drone is out of the geo fence, it will land now.
        if(pos_sender.check_failsafe() == 1)
        {
            Command_Now.command = Land;
        }

        //无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.command == Land)
        {
            Command_Now.command = Land;
        }

        switch (Command_Now.command)
        {
        case Idle:
            pos_sender.idle();
            break;
        case Takeoff:
            pos_sp = Eigen::Vector3d(Takeoff_position[0],Takeoff_position[1],Takeoff_position[2]+Takeoff_height);
            vel_sp = Eigen::Vector3d(0.0,0.0,0.0);
            yaw_sp = pos_sender.euler_fcu[2]; //rad

            pos_sender.send_pos_setpoint(pos_sp, yaw_sp);

            break;

        case Move_ENU:

            if( Command_Now.sub_mode == 0 )
            {
                pos_sp = Eigen::Vector3d(Command_Now.pos_sp[0],Command_Now.pos_sp[1],Command_Now.pos_sp[2]);
                yaw_sp = Command_Now.yaw_sp;

                pos_sender.send_pos_setpoint(pos_sp, yaw_sp);
            }
            else if( Command_Now.sub_mode == 3 )
            {
                vel_sp = Eigen::Vector3d(Command_Now.vel_sp[0],Command_Now.vel_sp[1],Command_Now.vel_sp[2]);

                pos_sender.send_vel_setpoint(vel_sp, yaw_sp);
            }

            break;

            //这里机体系只提供速度模式
        case Move_Body:

            vel_sp = Eigen::Vector3d(Command_Now.vel_sp[0],Command_Now.vel_sp[1],Command_Now.vel_sp[2]);

            yaw_sp = Command_Now.yaw_sp;

            pos_sender.send_vel_setpoint_body(vel_sp, yaw_sp);

            break;

        case Hold:
            if (Command_Last.command != Hold)
            {
                pos_sp = Eigen::Vector3d(pos_sender.pos_drone_fcu[0],pos_sender.pos_drone_fcu[1],pos_sender.pos_drone_fcu[2]);
                yaw_sp = pos_sender.euler_fcu[2];
            }

            pos_sender.send_pos_setpoint(pos_sp, yaw_sp);
            break;


        case Land:
            if (Command_Last.command != Land)
            {
                pos_sp = Eigen::Vector3d(pos_sender.pos_drone_fcu[0],pos_sender.pos_drone_fcu[1],Takeoff_position[2]);
                yaw_sp = pos_sender.euler_fcu[2];
            }

            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if(abs(pos_sender.pos_drone_fcu[2] - Takeoff_position[2]) < Disarm_height)
            {
                if(pos_sender.current_state.mode == "OFFBOARD")
                {
                    pos_sender.mode_cmd.request.custom_mode = "MANUAL";
                    pos_sender.set_mode_client.call(pos_sender.mode_cmd);
                }

                if(pos_sender.current_state.armed)
                {
                    pos_sender.arm_cmd.request.value = false;
                    pos_sender.arming_client.call(pos_sender.arm_cmd);

                }

                if (pos_sender.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {

                pos_sender.send_pos_setpoint(pos_sp, yaw_sp);
            }

            break;

        case Disarm:

            if(pos_sender.current_state.mode == "OFFBOARD")
            {
                pos_sender.mode_cmd.request.custom_mode = "MANUAL";
                pos_sender.set_mode_client.call(pos_sender.mode_cmd);
            }

            if(pos_sender.current_state.armed)
            {
                pos_sender.arm_cmd.request.value = false;
                pos_sender.arming_client.call(pos_sender.arm_cmd);

            }

            if (pos_sender.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        case Failsafe_land:

            break;

        }


        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;

}

// 【获取当前时间函数】 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.command)
    {
    case Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;
        break;
    case Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;
        break;
    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case Failsafe_land:
        cout << "Command: [ Failsafe_land ] " <<endl;
        break;
    case Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;
    case Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        break;

    }

    int sub_mode;
    sub_mode = Command_Now.sub_mode;

    if((sub_mode & 0b10) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_setpoint   : " << Command_Now.pos_sp[0] << " [ m ]"  << "  Y_setpoint : "<< Command_Now.pos_sp[1] << " [ m ]"<<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_setpoint   : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_setpoint : "<< Command_Now.vel_sp[1] << " [m/s]" <<endl;
    }

    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.pos_sp[2] << " [ m ]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control "<<endl;
        cout << "Z_setpoint   : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
    }

    cout << "Yaw_setpoint : "  << Command_Now.yaw_sp * 180/M_PI<< " [deg] " <<endl;
}
