/***************************************************************************************************************************
* px4_multidrone_pos_controller.cpp
*
* Author: Longhao Qian
*
* Update Time: 2020.10.30
*
* Introduction:  PX4 controller for multi drone implimentation. UAV is labeled as UAV#. UAV# is added to the preflex of very topic
*         1. get control command from /UAV# /px4_command/control_command topic. Message type:（ControlCommand.msg）
*         2. obtain drone state information from px4_multidrone_estimator
*         3. The position control is set to cascade_PID,
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
#include <payload_controller_JGCD.h>
/*--------------------------utility classes-----------------------*/
#include <px4_command_utils.h>
#include <px4_command/ControlCommand.h>
#include <px4_command/GeneralInfo.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/Trajectory.h>
#include <px4_command/Topic_for_log.h>
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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>> UAV command and state <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
static px4_command::ControlCommand Command_Now;
static px4_command::ControlCommand Command_Last;
static px4_command::ControlCommand Command_to_gs;
static px4_command::DroneState _DroneState;                         // drone state from estimator
static Eigen::Vector3d throttle_sp;
static px4_command::ControlOutput _ControlOutput;
static px4_command::AttitudeReference _AttitudeReference;           // attitude target sent to FCU
static float cur_time;
static px4_command::Topic_for_log _Topic_for_log;

/*--TO DO --- Auto Land*/

static float Takeoff_height;                                       //
static float Disarm_height;                                        //

static bool PrintState;
static bool isMulti;                                               // cooperative mode true for multi-drone, false for single drone
static bool isCorrectDrone;                                        // this is used in single-drone mode for determine whether the correct drone is used.
static int  CurrentdroneID;
static int  TargetdroneID;
//>>--------------------------  geographic fence --------------------------------<<
static Eigen::Vector2f geo_fence_x;
static Eigen::Vector2f geo_fence_y;
static Eigen::Vector2f geo_fence_z;

static Eigen::Vector3d Takeoff_position = Eigen::Vector3d(0.0,0.0,0.0);
static Eigen::Vector3d pos_drone_mocap;                             //无人机当前位置 (vicon)

/*--------------------- utility functions -------------------------------*/
bool CheckReferencePosition(const px4_command::ControlCommand& command_) {
    // check whether the position command is inside the safe zone
    if (command_.Reference_State.position_ref[0] < geo_fence_x[0] || command_.Reference_State.position_ref[0] > geo_fence_x[1] ||
        command_.Reference_State.position_ref[1] < geo_fence_y[0] || command_.Reference_State.position_ref[1] > geo_fence_y[1] ||
        command_.Reference_State.position_ref[2] < geo_fence_z[0] || command_.Reference_State.position_ref[2] > geo_fence_z[1]) {
        return false;
        ROS_WARN("Control command is out of the safe zone. ");
    } else {
        return true;
    }
}
void PrintParam() {
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Takeoff_height: "<< Takeoff_height<<" [m] "<<endl;
    cout << "Disarm_height : "<< Disarm_height <<" [m] "<<endl;
    cout << "geo_fence_x : "<< geo_fence_x[0] << " [m]  to  "<<geo_fence_x[1] << " [m]"<< endl;
    cout << "geo_fence_y : "<< geo_fence_y[0] << " [m]  to  "<<geo_fence_y[1] << " [m]"<< endl;
    cout << "geo_fence_z : "<< geo_fence_z[0] << " [m]  to  "<<geo_fence_z[1] << " [m]"<< endl;
    cout <<">>>>>>>>>>>>>>>>>>>> Cooperative Control Mode <<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if (isMulti) {
        cout <<  " This drone is in multi-drone mode! "<< endl;
    } else {
        cout <<  " This drone is in single-drone mode! "<< endl;
        if (!isCorrectDrone) {
            // send a warning if the target drone ID is not current drone ID
            cout <<"WARNING!! The designated drone ID is : "<< TargetdroneID <<". However, the current ID is : "<< CurrentdroneID <<endl;
            cout << "The payload controller can not be activated!" <<endl;
        } else {
            cout << "This is the correct drone for single-drone payload experiment! " <<endl;
        }
    }
    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<endl;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> call back functions <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Command_cb(const px4_command::ControlCommand::ConstPtr& msg) {
    Command_Now = *msg;
    // if "land"  is received, the drone does not respond to any other commands
    if(Command_Last.Mode == command_to_mavros_multidrone::Land) {
        Command_Now.Mode = command_to_mavros_multidrone::Land;
    } else {
        if (isMulti) {
            /* if the drone is in multi-drone mode, the drone does not respond to Payload_Stabilization_SingleUAV
             the drone will keep executing previous command */
            if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization_SingleUAV) {
                Command_Now = Command_Last;
            }
            /*TO DO, add a fleet status check*/

        } else {
            // if the drone is in single-drone mode, the drone should not respond to Payload_Stabilization command
            // the drone will keep executing previous move enu command
            if(isCorrectDrone) {
                // if is the correct drone, check the command type
                if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization) {
                    Command_Now = Command_Last;
                }
            } else {
                /* if this is not the correct drone, the drone should not respond neither
                Payload_Stabilization nor Payload_Stabilization_SingleUAV, therefore:
                */
                if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization) {
                    Command_Now = Command_Last;
                } else if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization_SingleUAV ) {
                    Command_Now = Command_Last;
                }
                // this way, the drone will only respond to Move ENU command
            }
        }
    }

    // Check for geo fence: If drone is out of the geo fence, it will land now.
    if(!CheckReferencePosition(Command_Now)) {
        // if the reference command is out of the safe range,
        Command_Now = Command_Last;
    }
}

void drone_state_cb(const px4_command::DroneState::ConstPtr& msg) {
    _DroneState = *msg;
    _DroneState.time_from_start = cur_time;
}

void SendGeneralInfoToGroundstation(ros::ServiceClient& client, px4_command::GeneralInfo& ParamSrv){
    bool isresponserecieved = false;
    ros::Time begin_time    = ros::Time::now();
    float last_time         = px4_command_utils::get_time_in_sec(begin_time);
    float cur_time          = last_time;
    ROS_INFO("Sending general information to ground station ...");
    while (!isresponserecieved) {
      // very 3 seconds, send a parameter service call to the ground station.
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        if(((int)(cur_time*10) % 30) == 0) {
            ROS_INFO("Waiting for response from ground station..., time elapsed %f [s]",cur_time);
            isresponserecieved = client.call(ParamSrv);
          }
    }
    ROS_INFO("General information sent to ground station !");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4_multidrone_pos_controller");
    ros::NodeHandle nh("~");

    /*----------- determine the  ID of the drone -----------------------------*/
    SubTopic px4_commmand_control_command;
    SubTopic px4_commmand_drone_state;
    PubTopic px4_command_topic_for_log;
    PubTopic px4_command_generalinfo;
    // add preflex:
    strcpy (px4_commmand_control_command.str,"/uav");
    strcpy (px4_commmand_drone_state.str,"/uav");
    strcpy (px4_command_topic_for_log.str,"/uav");
    strcpy (px4_command_generalinfo.str,"/uav");
    char ID[20];
    if ( argc > 1) {
    // if ID is specified as the second argument
        strcat (px4_commmand_control_command.str,argv[1]);
        strcat (px4_commmand_drone_state.str,argv[1]);
        strcat (px4_command_topic_for_log.str,argv[1]);
        strcat (px4_command_generalinfo.str,argv[1]);
        strcpy (ID,argv[1]);
        CurrentdroneID = *argv[1] - '0';
        ROS_INFO("UAV ID specified as: uav%s", argv[1]);
    } else {
        // if ID is not specified, then set the drone to UAV0
        strcat (px4_commmand_control_command.str,"0");
        strcat (px4_commmand_drone_state.str,"0");
        strcat (px4_command_topic_for_log.str,"0");
        strcat (px4_command_generalinfo.str,"0");
        strcpy (ID,"0");
        CurrentdroneID = 0;
        ROS_WARN("NO UAV ID specified, set ID to 0.");
    }
    strcat (px4_commmand_control_command.str,"/px4_command/control_command");
    strcat (px4_commmand_drone_state.str,"/px4_command/drone_state");
    strcat (px4_command_topic_for_log.str,"/px4_command/topic_for_log");
    strcat (px4_command_generalinfo.str,"/px4_command/generalinfo");

    ROS_INFO("Subscribe ControlCommand from: %s", px4_commmand_control_command.str);
    ROS_INFO("Subscribe DroneState from: %s", px4_commmand_drone_state.str);
    ROS_INFO("Publish Topic_for_log to: %s", px4_command_topic_for_log.str);
    ROS_INFO("Client call general info to: %s", px4_command_generalinfo.str);
    // 本话题来自根据需求自定义的上层模块，比如track_land.cpp 比如move.cpp
    ros::Subscriber Command_sub     = nh.subscribe<px4_command::ControlCommand>(px4_commmand_control_command.str, 100, Command_cb);
    // 本话题来自根据需求自定px4_pos_estimator.cpp
    ros::Subscriber drone_state_sub = nh.subscribe<px4_command::DroneState>(px4_commmand_drone_state.str, 100, drone_state_cb);
    // 发布log消息至ground_station.cpp
    ros::Publisher log_pub          = nh.advertise<px4_command::Topic_for_log>(px4_command_topic_for_log.str, 100);
    // ros client to send the primary Parameters
    ros::ServiceClient  clientSendParameter = nh.serviceClient<px4_command::GeneralInfo>(px4_command_generalinfo.str);

    // 参数读取
    nh.param<float>("Takeoff_height", Takeoff_height, 0.3);
    nh.param<float>("Disarm_height", Disarm_height, 0.15);
    nh.param<bool> ("PrintState", PrintState, false);
    nh.param<float>("DroneGeoFence/x_min", geo_fence_x[0], -1.2);
    nh.param<float>("DroneGeoFence/x_max", geo_fence_x[1], 1.2);
    nh.param<float>("DroneGeoFence/y_min", geo_fence_y[0], -0.9);
    nh.param<float>("DroneGeoFence/y_max", geo_fence_y[1], 0.9);
    nh.param<float>("DroneGeoFence/z_min", geo_fence_z[0], 0.0);
    nh.param<float>("DroneGeoFence/z_max", geo_fence_z[1], 2);
    nh.param<bool> ("CooperativeMode/isMulti",isMulti,false);
    nh.param<int>  ("CooperativeMode/droneID",TargetdroneID,0);

    if (TargetdroneID == CurrentdroneID) {
        isCorrectDrone = true;
    } else {
        isCorrectDrone = false;
    }

    // 位置控制一般选取为50Hz，主要取决于位置状态的更新频率
    ros::Rate rate(50.0);

    // communication with mavros
    command_to_mavros_multidrone _command_to_mavros(ID);
    // cpid is used for basic single UAV position control
    pos_controller_cascade_PID pos_controller_cascade_pid;
    ROS_INFO("cascade pid is used for single UAV control. ");
    pos_controller_cascade_pid.printf_param();
    // methods of payload stabilization with single UAVs
    pos_controller_TIE      pos_controller_tie(ID,nh);
    // methods of payload stabilization with multiple UAVs
    payload_controller_GNC  pos_controller_GNC(ID,nh);
    multidronepayload::payload_controller_JGCD pos_controller_JGCD(ID,nh);
    // pick control law will be specified in parameter files
    int SingleUAVPayloadController;
    int CooperativePayload;
    nh.param<int>("SinglePayloadController", SingleUAVPayloadController, 0);
    nh.param<int>("CooperativePayload", CooperativePayload, 0);
    px4_command::GeneralInfo ParamSrv;
    ParamSrv.request.TargetdroneID = TargetdroneID;
    ParamSrv.request.isMulti = isMulti;

    if(isMulti) {
        switch (CooperativePayload) {
            case 0: {
                pos_controller_GNC.ros_topic_setup(nh);
                pos_controller_GNC.printf_param();
                ParamSrv.request.controllername = "TCST2020";
                break;
            }
            case 1: {
                pos_controller_JGCD.ros_topic_setup(nh);
                pos_controller_JGCD.printf_param();
                ParamSrv.request.controllername = "JGCD2020";
                break;
            }
            default: {
                pos_controller_GNC.ros_topic_setup(nh);
                pos_controller_GNC.printf_param();
                ParamSrv.request.controllername = "TCST2020";
                break;
            }
        }
    } else {
        switch (SingleUAVPayloadController) {
            case 0: {
                pos_controller_tie.printf_param();
                ParamSrv.request.controllername = "TIE2019";
                break;
            }
            default: {
                pos_controller_tie.printf_param();
                ParamSrv.request.controllername = "TIE2019";
                break;
            }
        }
    }
    SendGeneralInfoToGroundstation(clientSendParameter, ParamSrv);
    /******-------------------print parameters ---------------------******/
    PrintParam();

    /* set the parameter field on the ground station */

    int check_flag;
    // check the data output on
    ROS_INFO("Please check the parameter and setting, enter 1 to continue, else for quit: ");
    cin >> check_flag;
    if(check_flag != 1) {
        ROS_WARN("Found something wrong? terminating node ...");
        return -1;
    }
    ROS_INFO("Parameter ok. Ready to start controller ...");
    // waiting for the
    for(int i = 0; i < 50; i++) {
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
    // Initialize command:  the initial state is idle
    Command_Now.Mode = command_to_mavros_multidrone::Idle;
    Command_Now.Command_ID = 0;
    Command_Now.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
    Command_Now.Reference_State.position_ref[math_utils::Vector_X] = 0.0;
    Command_Now.Reference_State.position_ref[math_utils::Vector_Y] = 0.0;
    Command_Now.Reference_State.position_ref[math_utils::Vector_Z] = 0.0;
    Command_Now.Reference_State.velocity_ref[math_utils::Vector_X] = 0.0;
    Command_Now.Reference_State.velocity_ref[math_utils::Vector_Y] = 0.0;
    Command_Now.Reference_State.velocity_ref[math_utils::Vector_Z] = 0.0;
    Command_Now.Reference_State.acceleration_ref[math_utils::Vector_X] = 0.0;
    Command_Now.Reference_State.acceleration_ref[math_utils::Vector_Y] = 0.0;
    Command_Now.Reference_State.acceleration_ref[math_utils::Vector_Z] = 0.0;
    Command_Now.Reference_State.yaw_ref = 0.0;

    // record the time
    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0;

    while(ros::ok()) {
        // get the current time and time stamp
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        ros::spinOnce();

        switch (Command_Now.Mode) {
        // idle mode: waiting for the command from ground station
        case command_to_mavros_multidrone::Idle:
            _command_to_mavros.idle();
            break;

        // take off mode 从摆放初始位置原地起飞至指定高度，偏航角也保持当前角度
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
            _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
            break;

        //  move ENU mode
        case command_to_mavros_multidrone::Move_ENU:
            Command_to_gs = Command_Now;
            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[0] = _ControlOutput.Throttle[0];
            throttle_sp[1] = _ControlOutput.Throttle[1];
            throttle_sp[2] = _ControlOutput.Throttle[2];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);
            _command_to_mavros.send_attitude_setpoint(_AttitudeReference);

            break;
        // land 降落。当前位置原地降落，降落后会自动上锁，且切换为mannual模式
        case command_to_mavros_multidrone::Land:
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros_multidrone::Land) {
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
            _command_to_mavros.send_attitude_setpoint(_AttitudeReference);

            //如果距离起飞高度小于15厘米，则直接上锁并切换为手动模式；
            if(_DroneState.position[2] - Takeoff_position[2] < Disarm_height)
            {

                ROS_INFO("Below landing height, disarming.");
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
        // Payload stabilization mode, single drone mode.
        case command_to_mavros_multidrone::Payload_Stabilization_SingleUAV: {
                Command_to_gs = Command_Now;
                _ControlOutput = pos_controller_tie.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

                if( pos_controller_tie.emergency_switch()){ 
                    // true means not in normal flight
                    Command_Now.Mode = command_to_mavros_multidrone::Payload_Land;
                } else {
                // false means ok
                    throttle_sp[math_utils::Vector_X] = _ControlOutput.Throttle[math_utils::Vector_X];
                    throttle_sp[math_utils::Vector_Y] = _ControlOutput.Throttle[math_utils::Vector_Y];
                    throttle_sp[math_utils::Vector_Z] = _ControlOutput.Throttle[math_utils::Vector_Z];

                    _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, 0);
                    _command_to_mavros.send_attitude_setpoint(_AttitudeReference);

                }
                break;
            }
        // payload stabilization mode, multi drone mode
        case command_to_mavros_multidrone::Payload_Stabilization: { 
                Command_to_gs = Command_Now;
                bool emergencyflag = false;
                switch (CooperativePayload) {
                    case 0: {
                        _ControlOutput = pos_controller_GNC.payload_controller(_DroneState, Command_to_gs.Reference_State, dt);
                        emergencyflag = pos_controller_GNC.emergency_switch();
                        break;
                    }
                    case 1: {
                        _ControlOutput = pos_controller_JGCD.payload_controller(_DroneState, Command_to_gs.Reference_State, dt);
                        emergencyflag = pos_controller_JGCD.emergency_switch();
                        break;
                    }
                    default: {
                        _ControlOutput = pos_controller_GNC.payload_controller(_DroneState, Command_to_gs.Reference_State, dt);
                        emergencyflag = pos_controller_GNC.emergency_switch();
                    break;
                    }
                }

                if(emergencyflag) {  
                    // true means not in normal flight
                    Command_Now.Mode = command_to_mavros_multidrone::Payload_Land;
                } else {
                    // false means
                    throttle_sp[math_utils::Vector_X] = _ControlOutput.Throttle[math_utils::Vector_X];
                    throttle_sp[math_utils::Vector_Y] = _ControlOutput.Throttle[math_utils::Vector_Y];
                    throttle_sp[math_utils::Vector_Z] = _ControlOutput.Throttle[math_utils::Vector_Z];
                    _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, 0.0); // set the yaw angle command to zero
                    _command_to_mavros.send_attitude_setpoint(_AttitudeReference);
                }
                break;
        }
        case command_to_mavros_multidrone::Payload_Land :{ 
            // end the payload stabilization mode and command the drone to hover at the same spot with an altitude of 0.7m
            Command_to_gs.Mode = Command_Now.Mode;
            Command_to_gs.Command_ID = Command_Now.Command_ID;
            if (Command_Last.Mode != command_to_mavros_multidrone::Payload_Land)
            {
                Command_to_gs.Reference_State.Sub_mode  = command_to_mavros_multidrone::XYZ_POS;
                Command_to_gs.Reference_State.position_ref[math_utils::Vector_X] = _DroneState.position[math_utils::Vector_X];
                Command_to_gs.Reference_State.position_ref[math_utils::Vector_Y] = _DroneState.position[math_utils::Vector_Y];
                Command_to_gs.Reference_State.position_ref[math_utils::Vector_Z] = 0.7; // set 0.7 for quadrotor hovering height
                Command_to_gs.Reference_State.velocity_ref[math_utils::Vector_X] = 0.0;
                Command_to_gs.Reference_State.velocity_ref[math_utils::Vector_Y] = 0.0;
                Command_to_gs.Reference_State.velocity_ref[math_utils::Vector_Z] = 0.0;
                Command_to_gs.Reference_State.acceleration_ref[math_utils::Vector_X] = 0.0;
                Command_to_gs.Reference_State.acceleration_ref[math_utils::Vector_Y] = 0.0;
                Command_to_gs.Reference_State.acceleration_ref[math_utils::Vector_Z] = 0.0;
                Command_to_gs.Reference_State.yaw_ref = _DroneState.attitude[2]; //rad
            }

            _ControlOutput = pos_controller_cascade_pid.pos_controller(_DroneState, Command_to_gs.Reference_State, dt);

            throttle_sp[math_utils::Vector_X] = _ControlOutput.Throttle[math_utils::Vector_X];
            throttle_sp[math_utils::Vector_Y] = _ControlOutput.Throttle[math_utils::Vector_Y];
            throttle_sp[math_utils::Vector_Z] = _ControlOutput.Throttle[math_utils::Vector_Z];

            _AttitudeReference = px4_command_utils::ThrottleToAttitude(throttle_sp, Command_to_gs.Reference_State.yaw_ref);
            _command_to_mavros.send_attitude_setpoint(_AttitudeReference);

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

        if(PrintState) {
            px4_command_utils::prinft_drone_state(_DroneState);
            px4_command_utils::printf_command_control(Command_to_gs);
            // print out cooperative payload control result
            if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization ) {
                switch (CooperativePayload) {
                    case 0: {
                        pos_controller_GNC.printf_result();
                        break;
                    }
                    case 1: {
                        pos_controller_JGCD.printf_result();
                        break;
                    }
                    default: {
                        pos_controller_GNC.printf_result();
                        break;
                    }
                }
            } else if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization_SingleUAV) {
                pos_controller_tie.printf_result();
            } else {
                // cout<<" >>>>>>>> NOT IN PAYLOAD STABILIAZATION MODE! <<<<<<<<<<"<<endl;
            }
            // print the control output to FCU
            px4_command_utils::prinft_attitude_reference(_AttitudeReference);
        }else if(((int)(cur_time*10) % 50) == 0)
        {
            ROS_INFO("Controller running normally. Time stamp: %f [s]",cur_time);
        }
        // send controller information for logging
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
