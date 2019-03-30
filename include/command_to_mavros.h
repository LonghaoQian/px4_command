/***************************************************************************************************************************
* command_to_mavros.h
*
* Author: Qyp
*
* Update Time: 2019.3.16
*
* Introduction:  Drone control command send class using Mavros package
*         1. Ref to the Mavros plugins (setpoint_raw, loca_position, imu and etc..)
*         2. Ref to the Offboard Flight task in PX4 code: https://github.com/PX4/Firmware/blob/master/src/lib/FlightTasks/tasks/Offboard/FlightTaskOffboard.cpp
*         3. Ref to the Mavlink module in PX4 code: https://github.com/PX4/Firmware/blob/master/src/modules/mavlink/mavlink_receiver.cpp
*         4. Ref to the position control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control
*         5. Ref to the attitude control module in PX4: https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control
*         6. 还需要考虑复合形式的输出情况
***************************************************************************************************************************/
#ifndef COMMAND_TO_MAVROS_H
#define COMMAND_TO_MAVROS_H

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
#include <bitset>

using namespace std;

namespace namespace_command_to_mavros {

class command_to_mavros
{
    public:
    //constructed function
    command_to_mavros(void):
        command_nh("~")
    {
        command_nh.param<float>("Takeoff_height", Takeoff_height, 1.0);

        pos_drone_fcu           = Eigen::Vector3d(0.0,0.0,0.0);
        vel_drone_fcu           = Eigen::Vector3d(0.0,0.0,0.0);
        q_fcu                   = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        Euler_fcu               = Eigen::Vector3d(0.0,0.0,0.0);
        pos_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        vel_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        pos_drone_fcu_target    = Eigen::Vector3d(0.0,0.0,0.0);
        accel_drone_fcu_target  = Eigen::Vector3d(0.0,0.0,0.0);
        q_fcu_target            = Eigen::Quaterniond(0.0,0.0,0.0,0.0);
        Euler_fcu_target        = Eigen::Vector3d(0.0,0.0,0.0);
        Thrust_target           = 0.0;

        Takeoff_position        = Eigen::Vector3d(0.0,0.0,0.0);
        Hold_position           = Eigen::Vector3d(0.0,0.0,0.0);

        type_mask_target        = 0;
        frame_target            = 0;

        state_sub = command_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &command_to_mavros::state_cb,this);

        position_sub = command_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &command_to_mavros::pos_cb,this);

        velocity_sub = command_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 100, &command_to_mavros::vel_cb,this);

        attitude_sub = command_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &command_to_mavros::att_cb,this);

        attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb,this);

        position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb,this);

        actuator_target_sub = command_nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/target_actuator_control", 10, &command_to_mavros::actuator_target_cb,this);

        setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

        arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    float Takeoff_height;

    //Takeoff Position of the Drone
    Eigen::Vector3d Takeoff_position;

    //Hold Position of the Drone (For hold mode in command.msg)
    Eigen::Vector3d Hold_position;

    //Current state of the drone
    mavros_msgs::State current_state;

    //Current pos of the drone
    Eigen::Vector3d pos_drone_fcu;
    //Current vel of the drone
    Eigen::Vector3d vel_drone_fcu;

    //Current att of the drone
    Eigen::Quaterniond q_fcu;
    Eigen::Vector3d Euler_fcu;

    //Target typemask [from fcu]
    int type_mask_target;

    int frame_target;

    //Target pos of the drone [from fcu]
    Eigen::Vector3d pos_drone_fcu_target;

    //Target vel of the drone [from fcu]
    Eigen::Vector3d vel_drone_fcu_target;

    //Target accel of the drone [from fcu]
    Eigen::Vector3d accel_drone_fcu_target;

    //Target att of the drone [from fcu]
    Eigen::Quaterniond q_fcu_target;
    Eigen::Vector3d Euler_fcu_target;

    //Target thrust of the drone [from fcu]
    float Thrust_target;

    mavros_msgs::ActuatorControl actuator_target;

    mavros_msgs::ActuatorControl actuator_setpoint;

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;

    ros::ServiceClient arming_client;

    ros::ServiceClient set_mode_client;

    void set_takeoff_position()
    {
        Takeoff_position = pos_drone_fcu;
    }

    //Takeoff to the default altitude, pls change the param in PX4: MIS_TAKEOFF_ALT
    void takeoff();

    //Land in current position in default velocity. Pls change the param in PX4 : MPC_LAND_SPEED
    void land();

    //Idle. Do nothing.
    void idle();

    //Loiter in the current position.
    void loiter();


    //Send pos_setpoint and yaw_setpoint in ENU frame to PX4
    void send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp);

    //Send vel_setpoint and yaw_setpoint in ENU frame to PX4
    void send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp);

    //Send pos_setpoint and yaw_setpoint in body frame to PX4
    void send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp);

    //Send accel_setpoint and yaw_setpoint in ENU frame to PX4
    void send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp);

    //Send actuator_setpoint to PX4
    void send_actuator_setpoint(Eigen::Vector4d actuator_sp);

    //Printf the parameters
    void printf_param();

    //Pringt the drone state
    void prinft_drone_state(float current_time);

    //Pringt the drone state2
    void prinft_drone_state2(float current_time);

    private:

        ros::NodeHandle command_nh;
        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber attitude_sub;
        ros::Subscriber attitude_target_sub;
        ros::Subscriber position_target_sub;
        ros::Subscriber actuator_target_sub;
        ros::Publisher setpoint_raw_local_pub;
        ros::Publisher actuator_setpoint_pub;

        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            current_state = *msg;
        }

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            pos_drone_fcu  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            vel_drone_fcu = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
        }

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to Euler Angles
            Euler_fcu = quaternion_to_euler(q_fcu);
        }

        void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
        {
            q_fcu_target = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            //Transform the Quaternion to Euler Angles
            Euler_fcu_target = quaternion_to_euler(q_fcu_target);

            Thrust_target = msg->thrust;
        }

        void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
        {
            type_mask_target = msg->type_mask;

            frame_target = msg->coordinate_frame;

            pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);

            vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

            accel_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
        }

        void actuator_target_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg)
        {
            actuator_target = *msg;
        }


};

void command_to_mavros::takeoff()
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0x1000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::land()
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0x2000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::loiter()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //Here pls ref to mavlink_receiver.cpp
    pos_setpoint.type_mask = 0x3000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0x4000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


void command_to_mavros::send_pos_setpoint(Eigen::Vector3d pos_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;
    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    //Bit 10 should set to 0, means is not force sp
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_BODY_NED = 8
    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_vel_setpoint(Eigen::Vector3d vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_vel_setpoint_body(Eigen::Vector3d vel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111000111;

    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_BODY_NED = 8
    pos_setpoint.coordinate_frame = 8;

    pos_setpoint.position.x = vel_sp[0];
    pos_setpoint.position.y = vel_sp[1];
    pos_setpoint.position.z = vel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_accel_setpoint(Eigen::Vector3d accel_sp, float yaw_sp)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100000111111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.acceleration_or_force.x = accel_sp[0];
    pos_setpoint.acceleration_or_force.y = accel_sp[1];
    pos_setpoint.acceleration_or_force.z = accel_sp[2];

    pos_setpoint.yaw = yaw_sp * M_PI/180;

    setpoint_raw_local_pub.publish(pos_setpoint);
}


void command_to_mavros::send_actuator_setpoint(Eigen::Vector4d actuator_sp)
{
    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = actuator_sp[0];
    actuator_setpoint.controls[1] = actuator_sp[1];
    actuator_setpoint.controls[2] = actuator_sp[2];
    actuator_setpoint.controls[3] = actuator_sp[3];
    actuator_setpoint.controls[4] = 0.0;
    actuator_setpoint.controls[5] = 0.0;
    actuator_setpoint.controls[6] = 0.0;
    actuator_setpoint.controls[7] = 0.0;

    actuator_setpoint_pub.publish(actuator_setpoint);
}

// 【打印参数函数】
void command_to_mavros::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "takeoff_height : "<< Takeoff_height << endl;

}

void command_to_mavros::prinft_drone_state(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout.setf(ios::fixed);

    cout << "Time: " << fixed <<setprecision(1)<< current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state.mode<<" ]   " <<endl;

    cout << "Position [X Y Z] : " << fixed <<setprecision(2)<< pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1]<<" [ m ] "<<pos_drone_fcu[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1]<<" [m/s] "<<vel_drone_fcu[2]<<" [m/s] "<<endl;

    cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180/M_PI <<" [deg] "<<Euler_fcu[1] * 180/M_PI << " [deg] "<< Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Acc_target [X Y Z] : "  << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;

    cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180/M_PI <<" [deg] "<<Euler_fcu_target[1] * 180/M_PI << " [deg] "<< Euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;

    cout << "actuator_target.group_mix : " << actuator_target.group_mix <<endl;

    cout << "actuator_target [0 1 2 3] : " << fixed <<setprecision(3)<< actuator_target.controls[0] << " [ m ] "<< actuator_target.controls[1] <<" [ m ] "<<actuator_target.controls[2]<<" [ m ] "<<actuator_target.controls[3] <<" [ m ] "<<endl;

    cout << "actuator_target [4 5 6 7] : " << fixed <<setprecision(3)<< actuator_target.controls[4] << " [ m ] "<< actuator_target.controls[5] <<" [ m ] "<<actuator_target.controls[6]<<" [ m ] "<<actuator_target.controls[7] <<" [ m ] "<<endl;

}

void command_to_mavros::prinft_drone_state2(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout.setf(ios::fixed);

    cout << "Time: " << fixed <<setprecision(1)<< current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state.mode<<" ]   " <<endl;

    cout << "Position [X Y Z] : " << fixed <<setprecision(2)<< pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1]<<" [ m ] "<<pos_drone_fcu[2]<<" [ m ] "<<endl;
    cout << "Velocity [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1]<<" [m/s] "<<vel_drone_fcu[2]<<" [m/s] "<<endl;
    cout << "Attitude [R P Y] : " << Euler_fcu[0] * 180/M_PI <<" [deg] "<<Euler_fcu[1] * 180/M_PI << " [deg] "<< Euler_fcu[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Pos_target [X Y Z] : "  << pos_drone_fcu_target[0] << " [ m ] "<< pos_drone_fcu_target[1]<<" [ m ] "<<pos_drone_fcu_target[2]<<" [ m ] "<<endl;
    cout << "Vel_target [X Y Z] : "  << vel_drone_fcu_target[0] << " [m/s] "<< vel_drone_fcu_target[1]<<" [m/s] "<<vel_drone_fcu_target[2]<<" [m/s] "<<endl;
    cout << "Acc_target [X Y Z] : "  << accel_drone_fcu_target[0] << " [m/s^2] "<< accel_drone_fcu_target[1]<<" [m/s^2] "<<accel_drone_fcu_target[2]<<" [m/s^2] "<<endl;

    cout << "Att_target [R P Y] : " << Euler_fcu_target[0] * 180/M_PI <<" [deg] "<<Euler_fcu_target[1] * 180/M_PI << " [deg] "<< Euler_fcu_target[2] * 180/M_PI<<" [deg] "<<endl;

    cout << "Thr_target [0 - 1] : " << Thrust_target <<endl;

}

}
#endif
