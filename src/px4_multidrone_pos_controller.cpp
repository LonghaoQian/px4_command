/***************************************************************************************************************************
* px4_multidrone_pos_controller.cpp
*
* Author: Longhao Qian
*
* Update Time: 2019.8.31
*
* Introduction:  PX4 controller for multi drone implimentation. UAV is IDed as UAV#. UAV# is added to the preflex of very topic
*         1. get control command from /UAV# /px4_command/control_command topic. Message type:（ControlCommand.msg）
*         2. 从command_from_mavros.h读取无人机的状态信息（DroneState.msg）。
*         3. 调用位置环控制算法，计算加速度控制量。可选择cascade_PID,
*         4. 通过command_to_mavros.h将计算出来的控制指令发送至飞控（通过mavros包）(mavros package will send the message to PX4 as Mavlink msg)
*         5. PX4 firmware will recieve the Mavlink msg by mavlink_receiver.cpp in mavlink module.
*         6. 发送相关信息至地面站节点(/UAV#/px4_command/attitude_reference)，供监控使用。
***************************************************************************************************************************/
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

#include <state_from_mavros_multidrone.h>
#include <command_to_mavros_multidrone.h>

#include <pos_controller_cascade_PID.h>
#include <pos_controller_TIE.h>
#include <payload_controller_GNC.h>
#include <payload_controller_TCST.h>
/*--------------------------utility classes-----------------------*/
#include <px4_command_utils.h>
#include <px4_command/ControlCommand.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/Trajectory.h>
#include <px4_command/Topic_for_log.h>
#include <px4_command/Trajectory.h>
#include <px4_command/ControlOutput.h>
#include <px4_command/PayloadPoseCommand.h>
using namespace std;

struct SubTopic
{
    char str[100];
};
struct PubTopic
{
    char str[100];
};
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV command and state <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
px4_command::ControlCommand Command_Now;                      //无人机当前执行命令
px4_command::ControlCommand Command_Last;                     //无人机上一条执行命令
px4_command::ControlCommand Command_to_gs;
px4_command::DroneState _DroneState;                         //无人机状态量
Eigen::Vector3d throttle_sp;
px4_command::ControlOutput _ControlOutput;
px4_command::AttitudeReference _AttitudeReference;           //位置控制器输出，即姿态环参考量
float cur_time;
px4_command::Topic_for_log _Topic_for_log;

float Takeoff_height;                                       //起飞高度
float Disarm_height;                                        //自动上锁高度
float Use_accel;                                            // 1 for use the accel command
int Flag_printf;

//>>--------------------------  geographic fence --------------------------------<<
Eigen::Vector2f geo_fence_x;
Eigen::Vector2f geo_fence_y;
Eigen::Vector2f geo_fence_z;

Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d pos_drone_mocap;                             //无人机当前位置 (vicon)

/*---------------------utility functions -------------------------------*/
int check_failsafe()
{
    if (_DroneState.position[0] < geo_fence_x[0] || _DroneState.position[0] > geo_fence_x[1] ||
        _DroneState.position[1] < geo_fence_y[0] || _DroneState.position[1] > geo_fence_y[1] ||
        _DroneState.position[2] < geo_fence_z[0] || _DroneState.position[2] > geo_fence_z[1]) {
        return 1;
        ROS_WARN("Out of the geo fence, the drone is landing");
    } else {
        return 0;
    }
}
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height: "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height : "<< Disarm_height <<" [m] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> call back functions <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
    // 无人机一旦接受到Land指令，则会屏蔽其他指令
    if(Command_Last.Mode == command_to_mavros_multidrone::Land)
    {
        Command_Now.Mode = command_to_mavros_multidrone::Land;
    }



    // Check for geo fence: If drone is out of the geo fence, it will land now.
    if(check_failsafe() == 1)
    {
        Command_Now.Mode = command_to_mavros_multidrone::Land;
    }
}

//void PayloadPoseTargetSub(const px4_command::PayloadPoseCommand& msg) {
 //   PayloadPoseTarget = *msg;
//}

void drone_state_cb(const px4_command::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    _DroneState.time_from_start = cur_time;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_multidrone_pos_controller");
    ros::NodeHandle nh("~");

    /*----------- determine the  ID of the drone -----------------------------*/
    SubTopic px4_commmand_control_command;
    SubTopic px4_commmand_drone_state;
    PubTopic px4_command_topic_for_log;
    // add preflex:
    strcpy (px4_commmand_control_command.str,"/uav");
    strcpy (px4_commmand_drone_state.str,"/uav");
    strcpy (px4_command_topic_for_log.str,"/uav");
    char ID[20];
    if ( argc > 1) {
    // if ID is specified as the second argument
        strcat (px4_commmand_control_command.str,argv[1]);
        strcat (px4_commmand_drone_state.str,argv[1]);
        strcat (px4_command_topic_for_log.str,argv[1]);
        strcpy (ID,argv[1]);
        ROS_INFO("UAV ID specified as: uav%s", argv[1]);
    } else {
        // if ID is not specified, then set the drone to UAV0
        strcat (px4_commmand_control_command.str,"0");
        strcat (px4_commmand_drone_state.str,"0");
        strcat (px4_command_topic_for_log.str,"0");
        strcpy (ID,"0");
        ROS_WARN("NO UAV ID specified, set ID to 0.");
    }
    strcat (px4_commmand_control_command.str,"/px4_command/control_command");
    strcat (px4_commmand_drone_state.str,"/px4_command/drone_state");
    strcat (px4_command_topic_for_log.str,"/px4_command/topic_for_log");

    ROS_INFO("Subscribe ControlCommand from: %s", px4_commmand_control_command.str);
    ROS_INFO("Subscribe DroneState from: %s", px4_commmand_drone_state.str);
    ROS_INFO("Publish Topic_for_log to: %s", px4_command_topic_for_log.str);

    // 本话题来自根据需求自定义的上层模块，比如track_land.cpp 比如move.cpp
    ros::Subscriber Command_sub = nh.subscribe<px4_command::ControlCommand>(px4_commmand_control_command.str, 100, Command_cb);

    // subscribe payload pose target
    //ros::Subscriber Payload_Command_sub = nh.subscribe<px4_command::PayloadPoseCommand>("/payload/px4_command/control_command",100,PayloadPoseTargetSub);

    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<px4_command::DroneState>(px4_commmand_drone_state.str, 100, drone_state_cb);

    // 发布log消息至ground_station.cpp
    ros::Publisher log_pub = nh.advertise<px4_command::Topic_for_log>(px4_command_topic_for_log.str, 100);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 0.3);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<float>("Use_accel", Use_accel, 0.0);
    nh.param<int>("Flag_printf", Flag_printf, 0.0);
    nh.param<float>("geo_fence/x_min", geo_fence_x[0], -1.2);
    nh.param<float>("geo_fence/x_max", geo_fence_x[1], 1.2);
    nh.param<float>("geo_fence/y_min", geo_fence_y[0], -0.9);
    nh.param<float>("geo_fence/y_max", geo_fence_y[1], 0.9);
    nh.param<float>("geo_fence/z_min", geo_fence_z[0], 0.2);
    nh.param<float>("geo_fence/z_max", geo_fence_z[1], 2);

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(50.0);

    // 用于与mavros通讯的类，通过mavros发送控制指令至飞控【本程序->mavros->飞控】
    /*TODO change this to multidrone case*/
    command_to_mavros_multidrone _command_to_mavros(ID);

    // cpid is used for single UAV position control
    pos_controller_cascade_PID pos_controller_cascade_pid;
    ROS_INFO("cascade pid is used for single UAV control. ");
    pos_controller_cascade_pid.printf_param();
    // methods of payload stabilization with single UAVs
    pos_controller_TIE      pos_controller_tie;
    // methods of payload stabilization with multiple UAVs
    /*TODO*/
    payload_controller_GNC  pos_controller_GNC(ID,nh);
    //payload_controller_TCST pos_controller_GNC(ID,nh);
    // pick control law will be specified in parameter files
    int SingleUAVPayloadController;
    int CooperativePayload;
    nh.param<int>("SinglePayloadController", SingleUAVPayloadController, 0);
    nh.param<int>("CooperativePayload", CooperativePayload, 0);
    switch (SingleUAVPayloadController) {
        case 0: {
            pos_controller_tie.printf_param();
            break;
        }
        default: {
            pos_controller_tie.printf_param();
            break;
        }
    }
    switch (CooperativePayload) {
        case 0: {
            pos_controller_GNC.printf_param();
            break;
        }
        case 1: {
            /*TODO: pos_controller_jgcd.printf_param();*/
            break;
        }
        case 2: {
            //pos_controller_gnc2019.printf_param();
            break;
        }
        default: {
            pos_controller_GNC.printf_param();
            break;
        }
    }

    /******-------------------print parameters ---------------------******/
    printf_param();

    /* set the parameter field on the ground station */
    
    int check_flag;
    // check the data output on 
    ROS_INFO("Please check the parameter and setting, enter 1 to continue, else for quit: ");
    cin >> check_flag;
    if(check_flag != 1)
    {
        ROS_WARN("Found something wrong? terminating node ...");
        return -1;
    }
    ROS_INFO("Parameter ok. Ready to start controller ...");
    // waiting for the 
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Set the takeoff position
    Takeoff_position[0] = _DroneState.position[0];
    Takeoff_position[1] = _DroneState.position[1];
    Takeoff_position[2] = _DroneState.position[2];
    // display takeoff position
    ROS_INFO("Takeoff Position X: %f [s]",Takeoff_position[0]);
    ROS_INFO("Takeoff Position Y: %f [s]",Takeoff_position[1]);
    ROS_INFO("Takeoff Position Z: %f [s]",Takeoff_position[2]);
    // Initialize command:  默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode = command_to_mavros_multidrone::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
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
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0;

    while(ros::ok())
    {
        // 当前时间
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        //执行回调函数
        ros::spinOnce();

        switch (Command_Now.Mode) {
        // 【Idle】 怠速旋转，此时可以切入offboard模式，但不会起飞。
        case command_to_mavros_multidrone::Idle:
            _command_to_mavros.idle();
            break;

        // 【Takeoff】 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
        case command_to_mavros_multidrone::Takeoff:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            Command_to_gs.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
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
            /*-----------   --------------*/

            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            }

            break;

        // 【Move_ENU】 ENU系移动。只有PID算法中才有追踪速度的选项，其他控制只能追踪位置
        case command_to_mavros_multidrone::Move_ENU:
            Command_to_gs = Command_Now;
            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5)
            {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            }
            break;
        // 【Land】 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case command_to_mavros_multidrone::Land:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros_multidrone::Land)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
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

            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5) {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else
            {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            }

            //如果距离起飞高度小于15厘米，则直接上锁并切换为手动模式；
            if(_DroneState.position[2] - Takeoff_position[2] < Disarm_height)
            {

                ROS_INFO("Below landing height, disarming.");

              /*  if(_DroneState.mode == "OFFBOARD") {
                    _command_to_mavros.mode_cmd.request.custom_mode = "MANUAL";
                    _command_to_mavros.set_mode_client.call(_command_to_mavros.mode_cmd);
                }*/

                if(_DroneState.armed) {
                    _command_to_mavros.arm_cmd.request.value = false;
                    _command_to_mavros.arming_client.call(_command_to_mavros.arm_cmd);
                }

                if (_command_to_mavros.arm_cmd.response.success)
                {
                    ROS_INFO("Disarm successfully!");
                }
            }


            break;

        case command_to_mavros_multidrone::Payload_Stabilization: 
            Command_to_gs = Command_Now;

            _ControlOutput = pos_controller_GNC.payload_controller(_DroneState, Command_to_gs.Reference_State,dt);
            
            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, 0);

            if (Use_accel > 0.5) {
                _command_to_mavros.send_accel_setpoint(throttle_sp,0);
            } else {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            }
            break;


        case command_to_mavros_multidrone::Payload_Land :{
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros_multidrone::Payload_Land)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[0] = _DroneState.position[0];
                Command_to_gs.Reference_State.position_ref[1] = _DroneState.position[1];
                Command_to_gs.Reference_State.position_ref[2] = 0.7; // set 0.7 for quadrotor hovering height
                Command_to_gs.Reference_State.velocity_ref[0] = 0;
                Command_to_gs.Reference_State.velocity_ref[1] = 0;
                Command_to_gs.Reference_State.velocity_ref[2] = 0;
                Command_to_gs.Reference_State.acceleration_ref[0] = 0;
                Command_to_gs.Reference_State.acceleration_ref[1] = 0;
                Command_to_gs.Reference_State.acceleration_ref[2] = 0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);

            if(Use_accel > 0.5) {
                _command_to_mavros.send_accel_setpoint(throttle_sp,Command_to_gs.Reference_State.yaw_ref);
            }else {
                _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            }
            break;
        }
        // 【Disarm】 紧急上锁。直接上锁，不建议使用，危险。
        case command_to_mavros_multidrone::Disarm:
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
        }

        if(Flag_printf == 1)
        {
            //cout <<">>>>>>>>>>>>>>>>>>>>>> px4_pos_controller <<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
            // 打印无人机状态
            px4_command_utils::prinft_drone_state(_DroneState);
            // 打印上层控制指令
            px4_command_utils::printf_command_control(Command_to_gs);
            // 打印位置控制器中间计算量
            pos_controller_GNC.printf_result();
            // 打印位置控制器输出结果
            px4_command_utils::prinft_attitude_reference(_AttitudeReference);
        }else if(((int)(cur_time*10) % 50) == 0)
        {
            ROS_INFO("Controller running normally. Time stamp: %f [s]",cur_time);
        }

        _Topic_for_log.header.stamp = ros::Time::now();
        _Topic_for_log.Drone_State = _DroneState;
        _Topic_for_log.Control_Command = Command_to_gs;
        _Topic_for_log.Attitude_Reference = _AttitudeReference;
        _Topic_for_log.Control_Output = _ControlOutput;

        log_pub.publish(_Topic_for_log);

        Command_Last = Command_Now;

        rate.sleep();
    }

    return 0;
}