/***************************************************************************************************************************
* qt_GUI_bridge.cpp
*
* Author: Longhao Qian
*
* Update Time: 2019.8.30
*
* Introduction:  a ros bridge between qt_ground_station and command_to_mavros
* This node is used for testing ros service call to send control command
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <px4_command/ControlCommand.h>
#include <command_to_mavros.h>

ros::Publisher move_pub;
px4_command::ControlCommand Command_Now;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_GUI_bridge");
    ros::NodeHandle nh;

    move_pub = nh.advertise<px4_command::ControlCommand>("/px4_command/control_command", 10);
    /*use ros service to */
    ros::ServiceClient CommandClient = n.serviceClient<px4_command::AddTwoInts>("add_two_ints");

    
    return 0;
}