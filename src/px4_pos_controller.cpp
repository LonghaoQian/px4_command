/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.5.9
*
* Introduction:  PX4 Position Controller using cascade PID method or PD+UDE or passivity
*         1. Subscribe command.msg from upper nodes (e.g. Target_tracking.cpp)
*         2. Calculate the accel_sp using pos_controller_PID.h(pos_controller_UDE.h pos_controller_passivity.h)
*         3. Send command to mavros package using command_to_mavros.h (mavros package will send the message to PX4 as Mavlink msg)
*         4. PX4 firmware will recieve the Mavlink msg by mavlink_receiver.cpp in mavlink module.
***************************************************************************************************************************/
#include <ros/ros.h>
#include <state_from_mavros.h>
#include <command_to_mavros.h>

#include <pos_controller_PID.h>
#include <pos_controller_UDE.h>
#include <pos_controller_Passivity.h>
#include <pos_controller_cascade_PID.h>
#include <pos_controller_NE.h>
#include <circle_trajectory.h>

#include <px4_command/ControlCommand.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <pos_controller_utils.h>
#include <px4_command/Trajectory.h>


#include <Eigen/Eigen>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;                      //无人机当前执行命令
px4_command::ControlCommand Command_Last;                     //无人机上一条执行命令

px4_command::ControlCommand Command_to_gs;

px4_command::DroneState _DroneState;                         //无人机状态量

Eigen::Vector3d accel_sp;
px4_command::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量

float Takeoff_height;
float Disarm_height;
float Use_mocap_raw;
float Use_accel;
//变量声明 - 其他变量
//Geigraphical fence 地理围栏
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d pos_drone_mocap;                             //无人机当前位置 (vicon)
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_ros_time(ros::Time begin);
int check_failsafe();
void prinft_command_state();
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
void printf_param();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}
void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 ENU系
    int optitrack_frame = 1; //Frame convention 0: Z-up -- 1: Y-up
    // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
    Eigen::Vector3d pos_drone_mocap_enu(-msg->pose.position.x,msg->pose.position.z,msg->pose.position.y);

    pos_drone_mocap = pos_drone_mocap_enu;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_controller");
    ros::NodeHandle nh("~");

    //【订阅】指令
    // 本话题来自根据需求自定义的上层模块，比如track_land.cpp 比如move.cpp
    ros::Subscriber Command_sub = nh.subscribe<px4_command::ControlCommand>("/px4/control_command", 10, Command_cb);

    // 订阅】来自mocap的数据 
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV/pose", 100, optitrack_cb);

    ros::Publisher att_ref_pub = nh.advertise<px4_command::AttitudeReference>("/px4_command/output", 10);

    ros::Publisher to_gs_pub = nh.advertise<px4_command::ControlCommand>("/px4/control_command_to_gs", 10);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 1.0);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Use_mocap_raw", Use_mocap_raw, 0.0);
    nh.param<float>("Use_accel", Use_accel, 0.0);
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -100.0);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 100.0);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -100.0);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 100.0);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], -100.0);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 100.0);

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(50.0);

    // 用于与mavros通讯的类，通过mavros接收来至飞控的消息【飞控->mavros->本程序】
    state_from_mavros _state_from_mavros;
    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    command_to_mavros _command_to_mavros;
    
    // 位置控制类 - 根据switch_ude选择其中一个使用，默认为PID
    pos_controller_cascade_PID pos_controller_cascade_pid;
    pos_controller_PID pos_controller_pid;
    pos_controller_UDE pos_controller_ude;
    pos_controller_passivity pos_controller_ps;
    pos_controller_NE pos_controller_ne;

    // 选择控制律
    int switch_ude;
    cout << "Please choose the controller: 0 for cascade_PID, 1 for PID, 2 for UDE, 3 for passivity, 4 for NE: "<<endl;
    cin >> switch_ude;

    if(switch_ude == 0)
    {
        pos_controller_cascade_pid.printf_param();
    }else if(switch_ude == 1)
    {
        pos_controller_pid.printf_param();
    }else if(switch_ude == 2)
    {
        pos_controller_ude.printf_param();
    }else if(switch_ude == 3)
    {
        pos_controller_ps.printf_param();
    }else if(switch_ude == 4)
    {
        pos_controller_ne.printf_param();
    }

    // 圆形轨迹追踪类
    Circle_Trajectory _Circle_Trajectory;
    float time_trajectory = 0.0;
    _Circle_Trajectory.printf_param();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 先读取一些飞控的数据
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set the takeoff position
    Takeoff_position[0] = _state_from_mavros._DroneState.position[0];
    Takeoff_position[1] = _state_from_mavros._DroneState.position[1];
    Takeoff_position[2] = _state_from_mavros._DroneState.position[2];

    // NE控制律需要设置起飞初始值
    if(switch_ude == 4)
    {
        pos_controller_ne.set_initial_pos(Takeoff_position);
    }

    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode = command_to_mavros::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
    Command_Now.Reference_State.position_ref[0] = 0;
    Command_Now.Reference_State.position_ref[1] = 0;
    Command_Now.Reference_State.position_ref[2] = 0;
    Command_Now.Reference_State.velocity_ref[0] = 0;
    Command_Now.Reference_State.velocity_ref[1] = 0;
    Command_Now.Reference_State.velocity_ref[2] = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref = 0;


    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = get_ros_time(begin_time);
    float dt = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        // 当前时间
        float cur_time = get_ros_time(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        // 获取当前无人机状态
        _DroneState.header.stamp = ros::Time::now();
        _DroneState.time_from_start = cur_time;

        _DroneState = _state_from_mavros._DroneState;

        if (Use_mocap_raw == 1)
        {
            for (int i=0;i<3;i++)
            {
                _DroneState.position[i] = pos_drone_mocap[i];
            }
        }

        // 打印无人机状态
        _state_from_mavros.prinft_drone_state(_DroneState);

        //Printf the command state
        //prinft_command_state();

        // if(switch_ude == 0)
        // {
        //     pos_controller_cascade_pid.printf_result();
        // }else if(switch_ude == 1)
        // {
        //     pos_controller_pid.printf_result();
        // }else if(switch_ude == 2)
        // {
        //     pos_controller_ude.printf_result();
        // }else if(switch_ude == 3)
        // {
        //     pos_controller_ps.printf_result();
        // }else if(switch_ude == 4)
        // {
        //     pos_controller_ne.printf_result();
        // }


        // 无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.Mode == command_to_mavros::Land)
        {
            Command_Now.Mode = command_to_mavros::Land;
        }

        // Check for geo fence: If drone is out of the geo fence, it will land now.
        if(check_failsafe() == 1)
        {
            Command_Now.Mode = command_to_mavros::Land;
        }

        att_ref_pub.publish(_AttitudeReference);

        to_gs_pub.publish(Command_to_gs);

        switch (Command_Now.Mode)
        {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case command_to_mavros::Idle:
            _command_to_mavros.idle();
            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
        case command_to_mavros::Takeoff:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
            Command_to_gs.Reference_State.position_ref[0] = Takeoff_position[0];
            Command_to_gs.Reference_State.position_ref[1] = Takeoff_position[1];
            Command_to_gs.Reference_State.position_ref[2] = Takeoff_position[2] + Takeoff_height;
            Command_to_gs.Reference_State.velocity_ref[0] = 0;
            Command_to_gs.Reference_State.velocity_ref[1] = 0;
            Command_to_gs.Reference_State.velocity_ref[2] = 0;
            Command_to_gs.Reference_State.acceleration_ref[0] = 0;
            Command_to_gs.Reference_State.acceleration_ref[1] = 0;
            Command_to_gs.Reference_State.acceleration_ref[2] = 0;
            Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad

            if(switch_ude == 0)
            {
                accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            
            _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);

            _AttitudeReference.thrust_sp[0] = accel_sp[0];
            _AttitudeReference.thrust_sp[1] = accel_sp[1];
            _AttitudeReference.thrust_sp[2] = accel_sp[2];

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            
            break;

        // 【Move_ENU】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case command_to_mavros::Move_ENU:
            Command_to_gs = Command_Now;

            if(switch_ude == 0)
            {
                accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }

            _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);

            _AttitudeReference.thrust_sp[0] = accel_sp[0];
            _AttitudeReference.thrust_sp[1] = accel_sp[1];
            _AttitudeReference.thrust_sp[2] = accel_sp[2];

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            break;

        // 【Move_Body】 机体系移动。
        case command_to_mavros::Move_Body:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            //只有在comid增加时才会进入解算 ： 机体系 至 惯性系
            if( Command_Now.Command_ID  >  Command_Last.Command_ID )
            {
                //xy velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b10 )
                {
                    float d_vel_body[2] = {Command_Now.Reference_State.velocity_ref[0], Command_Now.Reference_State.velocity_ref[1]};         //the desired xy velocity in Body Frame
                    float d_vel_enu[2];                                                           //the desired xy velocity in NED Frame

                    //根据无人机当前偏航角进行坐标系转换
                    rotation_yaw(_DroneState.attitude[2], d_vel_body, d_vel_enu);
                    Command_to_gs.Reference_State.position_ref[0] = 0;
                    Command_to_gs.Reference_State.position_ref[1] = 0;
                    Command_to_gs.Reference_State.velocity_ref[0] = d_vel_enu[0];
                    Command_to_gs.Reference_State.velocity_ref[1] = d_vel_enu[1];
                }
                //xy position mode
                else
                {
                    float d_pos_body[2] = {Command_Now.Reference_State.position_ref[0], Command_Now.Reference_State.position_ref[1]};         //the desired xy position in Body Frame
                    float d_pos_enu[2];                                                           //the desired xy position in enu Frame (The origin point is the drone)
                    rotation_yaw(_DroneState.attitude[2], d_pos_body, d_pos_enu);

                    Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0] + d_pos_enu[0];
                    Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1] + d_pos_enu[1];
                    Command_to_gs.Reference_State.velocity_ref[0] = 0;
                    Command_to_gs.Reference_State.velocity_ref[1] = 0;
                }

                //z velocity mode
                if( Command_Now.Reference_State.Sub_mode  & 0b01 )
                {
                    Command_to_gs.Reference_State.position_ref[2] = 0;
                    Command_to_gs.Reference_State.velocity_ref[2] = Command_Now.Reference_State.velocity_ref[2];
                }
                //z posiiton mode
                {
                    Command_to_gs.Reference_State.position_ref[2] = _DroneState.position[2] + Command_Now.Reference_State.position_ref[2];
                    Command_to_gs.Reference_State.velocity_ref[2] = 0; 
                }

                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2] + Command_Now.Reference_State.yaw_ref;

                float d_acc_body[2] = {Command_Now.Reference_State.acceleration_ref[0], Command_Now.Reference_State.acceleration_ref[1]};       
                float d_acc_enu[2]; 

                rotation_yaw(_DroneState.attitude[2], d_acc_body, d_acc_enu);
                Command_to_gs.Reference_State.acceleration_ref[0] = d_acc_enu[0];
                Command_to_gs.Reference_State.acceleration_ref[1] = d_acc_enu[1];
                Command_to_gs.Reference_State.acceleration_ref[2] = Command_Now.Reference_State.acceleration_ref[2];

            }

            if(switch_ude == 0)
            {
                accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);

            _AttitudeReference.thrust_sp[0] = accel_sp[0];
            _AttitudeReference.thrust_sp[1] = accel_sp[1];
            _AttitudeReference.thrust_sp[2] = accel_sp[2];


            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }

            break;

        // 【Hold】 悬停。当前位置悬停
        case command_to_mavros::Hold:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros::Hold)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_to_gs.Reference_State.position_ref[2] = _DroneState.position[2];
                Command_to_gs.Reference_State.velocity_ref[0] = 0;
                Command_to_gs.Reference_State.velocity_ref[1] = 0;
                Command_to_gs.Reference_State.velocity_ref[2] = 0;
                Command_to_gs.Reference_State.acceleration_ref[0] = 0;
                Command_to_gs.Reference_State.acceleration_ref[1] = 0;
                Command_to_gs.Reference_State.acceleration_ref[2] = 0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            if(switch_ude == 0)
            {
                accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }
            _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);

            _AttitudeReference.thrust_sp[0] = accel_sp[0];
            _AttitudeReference.thrust_sp[1] = accel_sp[1];
            _AttitudeReference.thrust_sp[2] = accel_sp[2];


            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            break;

        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case command_to_mavros::Land:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros::Land)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_to_gs.Reference_State.position_ref[2] = Takeoff_position[2];
                Command_to_gs.Reference_State.velocity_ref[0] = 0;
                Command_to_gs.Reference_State.velocity_ref[1] = 0;
                Command_to_gs.Reference_State.velocity_ref[2] = 0;
                Command_to_gs.Reference_State.acceleration_ref[0] = 0;
                Command_to_gs.Reference_State.acceleration_ref[1] = 0;
                Command_to_gs.Reference_State.acceleration_ref[2] = 0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            //如果距离起飞高度小于10厘米，则直接上锁并切换为手动模式；
            if(abs(_DroneState.position[2] - Takeoff_position[2]) < Disarm_height)
            {
                if(_DroneState.mode == "OFFBOARD")
                {
                    _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                }

                if(_DroneState.armed)
                {
                    _command_to_mavros.arm_cmd.request.value = false;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);

                }

                if (_command_to_mavros.arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {
                if(switch_ude == 0)
                {
                    accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 1)
                {
                    accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 2)
                {
                    accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 3)
                {
                    accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }else if(switch_ude == 4)
                {
                    accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
                }

                _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);
                _AttitudeReference.thrust_sp[0] = accel_sp[0];
                _AttitudeReference.thrust_sp[1] = accel_sp[1];
                _AttitudeReference.thrust_sp[2] = accel_sp[2];


                if(Use_accel > 0.5)
                {
                    _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
                }else
                {
                    _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
                }
             }


            break;

        // 【Disarm】 紧急上锁。直接上锁，不建议使用，危险。
        case command_to_mavros::Disarm:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if(_DroneState.mode == "OFFBOARD")
            {
                _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
            }

            if(_DroneState.armed)
            {
                _command_to_mavros.arm_cmd.request.value = false;
                _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);

            }

            if (_command_to_mavros.arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        // 【Failsafe_land】 暂空。可进行自定义
        case command_to_mavros::Failsafe_land:
            break;
        
        // Trajectory_Tracking 轨迹追踪控制，与上述追踪点或者追踪速度不同，此时期望输入为一段轨迹
        case command_to_mavros::Trajectory_Tracking:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            
            if (Command_Last.Mode != command_to_mavros::Trajectory_Tracking)
            {
                time_trajectory = 0.0;
            }

            time_trajectory = time_trajectory + dt;

            Command_to_gs.Reference_State = _Circle_Trajectory.Circle_trajectory_generation(time_trajectory);

            _Circle_Trajectory.printf_result();

            if(switch_ude == 0)
            {
                accel_sp = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 1)
            {
                accel_sp = pos_controller_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 2)
            {
                accel_sp = pos_controller_ude.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 3)
            {
                accel_sp = pos_controller_ps.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }else if(switch_ude == 4)
            {
                accel_sp = pos_controller_ne.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);
            }

            _AttitudeReference = pos_controller_utils::thrustToAttitude(accel_sp, Command_to_gs.Reference_State.yaw_ref);

            _AttitudeReference.thrust_sp[0] = accel_sp[0];
            _AttitudeReference.thrust_sp[1] = accel_sp[1];
            _AttitudeReference.thrust_sp[2] = accel_sp[2];

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(accel_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);            
            }
            
            // Quit 
            if (time_trajectory >= _Circle_Trajectory.time_total)
            {
                Command_Now.Mode = command_to_mavros::Hold;
            }

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
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>Control Command Mode<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    switch(Command_Now.Mode)
    {
    case command_to_mavros::Move_ENU:
        cout << "Command: [ Move_ENU ] " <<endl;

        break;
    case command_to_mavros::Move_Body:
        cout << "Command: [ Move_Body ] " <<endl;
        break;

    case command_to_mavros::Hold:
        cout << "Command: [ Hold ] " <<endl;
        cout << "Hold Position [X Y Z] : " << Command_to_gs.Reference_State.position_ref[0] << " [ m ] "<< Command_to_gs.Reference_State.position_ref[1]<<" [ m ] "<< Command_to_gs.Reference_State.position_ref[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << Command_to_gs.Reference_State.yaw_ref* 180/M_PI << " [deg] " <<endl;
        break;

    case command_to_mavros::Land:
        cout << "Command: [ Land ] " <<endl;
        cout << "Land Position [X Y Z] : " << Command_to_gs.Reference_State.position_ref[0] << " [ m ] "<< Command_to_gs.Reference_State.position_ref[1]<<" [ m ] "<< Command_to_gs.Reference_State.position_ref[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << Command_to_gs.Reference_State.yaw_ref* 180/M_PI << " [deg] " <<endl;
        break;

    case command_to_mavros::Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;

    case command_to_mavros::Failsafe_land:
        cout << "Command: [ Failsafe_land ] " <<endl;
        break;

    case command_to_mavros::Idle:
        cout << "Command: [ Idle ] " <<endl;
        break;

    case command_to_mavros::Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        cout << "Takeoff Position [X Y Z] : " << Command_to_gs.Reference_State.position_ref[0] << " [ m ] "<< Command_to_gs.Reference_State.position_ref[1]<<" [ m ] "<< Command_to_gs.Reference_State.position_ref[2]<<" [ m ] "<<endl;
        cout << "Yaw_setpoint : "  << Command_to_gs.Reference_State.yaw_ref* 180/M_PI << " [deg] " <<endl;
        break;
    }



}
// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height: "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height : "<< Disarm_height <<" [m] "<<endl;
    cout << "Use_mocap_raw : "<< Use_mocap_raw <<" [1 for use mocap raw data] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;

}

int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1])
    {
        return 1;
        cout << "Out of the geo fence, the drone is landing... "<< endl;
    }
    else{
        return 0;
    }
}

    //     cout << "Command: [ Move_Body ] " <<endl;
        
    //     if((Sub_mode & 0b10) == 0) //xy channel
    //     {
    //         cout << "Submode: xy position control "<<endl;
    //     }
    //     else{
    //         cout << "Submode: xy velocity control "<<endl;
    //     }

    //     if((Sub_mode & 0b01) == 0) //z channel
    //     {
    //         cout << "Submode:  z position control "<<endl;
    //     }
    //     else
    //     {
    //         cout << "Submode:  z velocity control "<<endl;
    //     }

    //     cout << "Pos_ref [XYZ]: " << Command_Now.Reference_State.position_ref[0] << " [ m ]" << Command_Now.Reference_State.position_ref[1] << " [ m ]"<< Command_Now.Reference_State.position_ref[2] << " [ m ]" << endl;
    //     cout << "Vel_ref [XYZ]: " << Command_Now.Reference_State.velocity_ref[0] << " [m/s]" << Command_Now.Reference_State.velocity_ref[1] << " [m/s]" << Command_Now.Reference_State.velocity_ref[2] << " [m/s]" <<endl;
    //     cout << "Vel_ref [XYZ]: " << Command_Now.Reference_State.acceleration_ref[0] << " [m/s^2]" << Command_Now.Reference_State.acceleration_ref[1] << " [m/s^2]" << Command_Now.Reference_State.acceleration_ref[2] << " [m/s^2]" <<endl;
    //     cout << "Yaw_setpoint : " << Command_Now.Reference_State.yaw_ref * 180/M_PI << " [deg] " <<endl;
