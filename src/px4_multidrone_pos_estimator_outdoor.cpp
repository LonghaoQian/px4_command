/***************************************************************************************************************************
 * px4_multidrone_pos_estimator_outdoor.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2020.10.31
 *
 * Get drone position, velocity and payload visual measurement from gps and camera
 *      1. subscrbe the filtered drone position and velocity from ekf
 *      2. publish the drone velocity and position and payload estimation 
 *      3. service call for setting the gps home position
 *
***************************************************************************************************************************/
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Eigen>
// user defined class
#include <state_from_mavros_multidrone.h>
#include <math_utils.h>
#include <px4_command_utils.h>
// custom msg
#include <px4_command/DroneState.h>
#include <px4_command/HomePosition.h>
// custom srv
#include <px4_command/SetHome.h>
// ros class and msgs
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

void GetGPSPos(const nav_msgs::Odometry::ConstPtr& msg, px4_command::DroneState* dronestate){
    dronestate->position[math_utils::Vector_X] = msg->pose.pose.position.x;
    dronestate->position[math_utils::Vector_Y] = msg->pose.pose.position.y;
    dronestate->position[math_utils::Vector_Z] = msg->pose.pose.position.z;
    dronestate->velocity[math_utils::Vector_X] = msg->twist.twist.linear.x;
    dronestate->velocity[math_utils::Vector_Y] = msg->twist.twist.linear.y;
    dronestate->velocity[math_utils::Vector_Z] = - msg->twist.twist.linear.z; // this is a potential bug in mavros node. (should be ENU)
}

void GetUAVBattery(const sensor_msgs::BatteryState::ConstPtr &msg, px4_command::DroneState* dronestate) {
    dronestate->battery_voltage   = msg->voltage;
    dronestate->battery_remaining = msg->percentage;
}

void GetMavrosState(const mavros_msgs::State::ConstPtr &msg, px4_command::DroneState* dronestate) {
    dronestate->connected = msg->connected;
    dronestate->armed = msg->armed;
    dronestate->mode = msg->mode;
}

void GetDroneAttitude(const sensor_msgs::Imu::ConstPtr& msg,  px4_command::DroneState* _DroneState) {
    Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    //Transform the Quaternion to euler Angles
    Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
    _DroneState->attitude_q.w = q_fcu.w();
    _DroneState->attitude_q.x = q_fcu.x();
    _DroneState->attitude_q.y = q_fcu.y();
    _DroneState->attitude_q.z = q_fcu.z();
    _DroneState->attitude[math_utils::EULER_ROLL]  = euler_fcu[math_utils::EULER_ROLL];
    _DroneState->attitude[math_utils::EULER_PITCH] = euler_fcu[math_utils::EULER_PITCH];
    _DroneState->attitude[math_utils::EULER_YAW]   = euler_fcu[math_utils::EULER_YAW];
    _DroneState->attitude_rate[math_utils::Vector_X] = msg->angular_velocity.x;
    _DroneState->attitude_rate[math_utils::Vector_Y] = msg->angular_velocity.y;
    _DroneState->attitude_rate[math_utils::Vector_Z] = msg->angular_velocity.z;
    _DroneState->acceleration[math_utils::Vector_X] = msg->linear_acceleration.x;
    _DroneState->acceleration[math_utils::Vector_Y] = msg->linear_acceleration.y;
    _DroneState->acceleration[math_utils::Vector_Z] = msg->linear_acceleration.z;
}

void GetDroneLongitudeLatitude(const sensor_msgs::NavSatFix::ConstPtr& msg, px4_command::DroneState* _DroneState) {
    _DroneState->longitude = msg->longitude;
    _DroneState->latitude  = msg->latitude;
}


bool ResponseToSetGPSHomeCall(px4_command::SetHome::Request& req, 
                              px4_command::SetHome::Response& res,
                              ros::Publisher* const pub, 
                              px4_command::HomePosition* const msg){
    msg->geo.latitude  = req.latitude;
    msg->geo.longitude = req.longitude;
    msg->geo.altitude  = req.altitude;
    msg->orientation.x = 1.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation.w = 0.0;
    (*pub).publish(*msg);
    res.homeset_ok = true;
    return true;
}


void GetVisionFeedback(const geometry_msgs::PoseStamped::ConstPtr& msg,  Eigen::Vector3f* payloadposition){
    (*payloadposition)(math_utils::Vector_X) = msg->pose.position.x;
    (*payloadposition)(math_utils::Vector_Y) = msg->pose.position.y;
    (*payloadposition)(math_utils::Vector_Z) = msg->pose.position.z;
}

void DisplayVisionResults(const Eigen::Vector3f& payloadposition, 
                          const Eigen::Vector3f& payloadvelocity, 
                          const Eigen::Vector3f& payloadpositionbody,
                          const Eigen::Vector3f& payloadvelocitybody,
                          const float& sampletime){

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    cout<<setprecision(2);// number of decimals 
    std::cout<< "----------------------- Payload Info ------------------ \n";
    std::cout<< "Payload inertial position X: " << payloadposition(math_utils::Vector_X) << " (m), Y: " << payloadposition(math_utils::Vector_Y)
    <<" (m), Z: "<< payloadposition(math_utils::Vector_Z) << " (m). \n";
     std::cout<< "Payload inertial velocity X: " << payloadvelocity(math_utils::Vector_X) * 100.0 << " (cm/s), Y: " << payloadvelocity(math_utils::Vector_Y)* 100.0
    <<" (cm/s), Z: "<< payloadvelocity(math_utils::Vector_Z)* 100.0 << " (cm/s). \n";   
    std::cout<< "The sample time is: " << sampletime << " (s) \n";
    std::cout<< "Payload body position X: " << payloadpositionbody(math_utils::Vector_X) << " (m), Y: " << payloadpositionbody(math_utils::Vector_Y)
    <<" (m), Z: "<< payloadpositionbody(math_utils::Vector_Z) << " (m). \n";
     std::cout<< "Payload body velocity X: " << payloadvelocitybody(math_utils::Vector_X)* 100.0<< " (cm/s), Y: " << payloadvelocitybody(math_utils::Vector_Y)* 100.0
    <<" (cm/s), Z: "<< payloadvelocitybody(math_utils::Vector_Z)* 100.0 << " (cm/s). \n";   
    std::cout<< "The sample time is: " << sampletime << " (s) \n";
    std::cout<< "------------------End of Payload Info ------------------ \n";
}

int main(int argc, char **argv){

    ros::init(argc, argv, "px4_multidrone_pos_estimator_outdoor");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    // define variables
    namespace arg = std::placeholders;
    px4_command::DroneState DroneStateData;  
    px4_command::HomePosition GPSGlobalHome;
    bool IsPayloadVisionReceived = false;                         // flag determine whether the vision feedback is updated
    bool IsVisionFeedbackUsed    = false;
    bool IsVisionMulti           = false;                        
    nh.param<bool>("IsVisionUsed", IsVisionFeedbackUsed , false); // whether vision feedback is used
    nh.param<bool>("IsVisionMulti", IsVisionMulti, false);        // whether vision feedback is used in multidrone mode
    //ros::Subscriber* Subvision = NULL; To modify this later
    Eigen::Vector3f PayloadPosition;
    Eigen::Vector3f QuadrotorPositionInertial;
    Eigen::Vector3f QuadrotorVelocityInertial;
    Eigen::Vector3f PayloadPositionInertial;
    Eigen::Vector3f PayloadVelocityInertial;
    Eigen::Vector3f PayloadVelocity;
    Eigen::Vector3f PreviousPayloadPosition;
    Eigen::Vector3f PreviousPayloadVelocity;
    Eigen::Vector3f PayloadAngularVelocity;
    Eigen::Matrix3f R_IB;
    Eigen::Vector4f Quad_Drone;
    Eigen::Matrix3f OmegaCross;
    Eigen::Vector3f Omega;
    Omega.setZero();
    PayloadPositionInertial.setZero();
    PayloadVelocityInertial.setZero();
    PayloadPosition.setZero();
    PayloadVelocity.setZero();
    PreviousPayloadPosition.setZero();
    PreviousPayloadVelocity.setZero();
    PayloadAngularVelocity.setZero();
    float beta = 0.8; 
    float Ts = 0.02;  
    // define the drone ID and ros topics:
    std::string TopicMavrosGPSPos = "/uav";
    std::string TopicMavrosGPSGlobal = "/uav";
    std::string TopicVisionPayload = "/uav";
    std::string TopicPx4CommandDroneState = "/uav";
    std::string TopicMavrosIMU = "/uav";
    std::string TopicMavrosState = "/uav";
    std::string TopicMavrosBattery = "/uav";
    std::string TopicMavrosGPSHome = "/uav";
    std::string TopicGCSGPSHome = "/uav";
    std::string TopicVisionFeedback = "/";// TO DO: add a uav tag later
    char* droneID = NULL;
    char droneDefaultID = '0';
    if ( argc > 1) {    // if ID is specified as the second argument 
        ROS_INFO("UAV ID specified as: UAV%s", argv[1]);
        droneID = argv[1];
    } else {
        // if ID is not specified, then set the drone to UAV0
        droneID = &droneDefaultID;
        ROS_WARN("NO UAV ID is specified, set the ID to 0!");
    }
    // add uav prefixes to topic strings 
    TopicMavrosGPSPos.push_back(*droneID);
    TopicMavrosGPSGlobal.push_back(*droneID);
    TopicVisionPayload.push_back(*droneID); // TO DO
    TopicPx4CommandDroneState.push_back(*droneID);
    TopicMavrosIMU.push_back(*droneID);
    TopicMavrosState.push_back(*droneID);
    TopicMavrosBattery.push_back(*droneID);
    TopicMavrosGPSHome.push_back(*droneID);
    TopicGCSGPSHome.push_back(*droneID);
    TopicVisionFeedback.push_back(*droneID);
    // add topic names to topic strings
    TopicMavrosGPSPos         += "/mavros/global_position/local";
    TopicMavrosGPSGlobal      += "/mavros/global_position/global";   
    TopicPx4CommandDroneState += "/px4_command/drone_state";
    TopicMavrosIMU            += "/mavros/imu/data";
    TopicMavrosState          += "/mavros/state";
    TopicMavrosBattery        += "/mavros/battery";
    TopicMavrosGPSHome        += "/mavros/global_position/home";
    TopicGCSGPSHome           += "/px4_command/globalhome";
    TopicVisionFeedback       += "/px4_command/vision";// To do
    // define publishers and subscribers
    // publish drone state to controller and ground statioin
    ros::Publisher  PubDroneState     = nh.advertise<px4_command::DroneState>(TopicPx4CommandDroneState, 100);
    // publish gps home position to mavros to reset the local origin
    ros::Publisher  PubGPSHome        = nh.advertise<px4_command::HomePosition>(TopicMavrosGPSHome, 100);
    // get the global latitude and longitude from mavros
    ros::Subscriber SubGPSGlobal 	  = nh.subscribe<sensor_msgs::NavSatFix>(TopicMavrosGPSGlobal, 
                                                                             100, 
                                                                             std::bind(&GetDroneLongitudeLatitude, arg::_1, &DroneStateData));
    // get the local position in ENU frame from mavros
    ros::Subscriber SubGPSPosition    = nh.subscribe<nav_msgs::Odometry>(TopicMavrosGPSPos, 
                                                                         100, 
                                                                         std::bind(&GetGPSPos, arg::_1, &DroneStateData));
    // get the battery status from mavros
    ros::Subscriber SubMavrosBattery  = nh.subscribe<sensor_msgs::BatteryState> (TopicMavrosBattery,
                                                                                 100,
                                                                                 std::bind(&GetUAVBattery, arg::_1, &DroneStateData));
    // get the connection state from mavros
    ros::Subscriber SubMavrosState    = nh.subscribe<mavros_msgs::State>(TopicMavrosState, 
                                                                         100, 
                                                                         std::bind(&GetMavrosState, arg::_1,  &DroneStateData));
    // get the attitude and accelerations from IMU
    ros::Subscriber SubDroneAttitude  = nh.subscribe<sensor_msgs::Imu>(TopicMavrosIMU, 
                                                                       100, 
                                                                       std::bind(&GetDroneAttitude, arg::_1,  &DroneStateData));
    // service call to set the gps home position
     ros::ServiceServer serverAction   = nh.advertiseService<px4_command::SetHome::Request, px4_command::SetHome::Response>(TopicGCSGPSHome,
                                                             std::bind(&ResponseToSetGPSHomeCall, arg::_1, arg::_2,&PubGPSHome,&GPSGlobalHome));
    // get the vision feedback from the camera

    // ros vision subscriber
    //Subvision = new ros::Subscriber Subvision;
    ros::Subscriber Subvision =  nh.subscribe<geometry_msgs::PoseStamped>("/pose_tag_body",
                                                                          100,
                                                                           std::bind(&GetVisionFeedback, arg::_1, &PayloadPosition));


    // display topics:
    ROS_INFO("Subscribe uav GPS position and velocity from: %s", TopicMavrosGPSPos.c_str());
    ROS_INFO("Subscribe mavros_msgs::State from: %s", TopicMavrosState.c_str());
    ROS_INFO("Subscribe IMU from: %s", TopicMavrosIMU.c_str()); 
    ROS_INFO("Subscribe battery info from: %s", TopicMavrosBattery.c_str()); 
    ROS_INFO("Subscribe global latitude and longitude from: %s",TopicMavrosGPSGlobal.c_str());
    ROS_INFO("Publish DroneState to: %s", TopicPx4CommandDroneState.c_str());
    ROS_INFO("Publish gps home position to: %s",TopicMavrosGPSHome.c_str());
    ROS_INFO("Service server for set gps home, from: %s", TopicGCSGPSHome.c_str());
    ROS_INFO("Start the estimator...");

   
    ros::Time BeginTime  = ros::Time::now();
    float LastTime = px4_command_utils::get_time_in_sec(BeginTime);
    float CurrentTime = 0.0;
    while(ros::ok()) {
        ros::spinOnce();
         // calculate the time interval
        CurrentTime = px4_command_utils::get_time_in_sec(BeginTime);
        Ts = constrain_function2(CurrentTime - LastTime, 0.01, 0.03);// update sample time
        LastTime = CurrentTime;// save current time as last time
        // calculate the speed of the payload using a high-pass filter

        QuadrotorPositionInertial(math_utils::Vector_X) = DroneStateData.position[math_utils::Vector_X];
        QuadrotorPositionInertial(math_utils::Vector_Y) = DroneStateData.position[math_utils::Vector_Y];
        QuadrotorPositionInertial(math_utils::Vector_Z) = DroneStateData.position[math_utils::Vector_Z];

        QuadrotorVelocityInertial(math_utils::Vector_X) = DroneStateData.velocity[math_utils::Vector_X];
        QuadrotorVelocityInertial(math_utils::Vector_Y) = DroneStateData.velocity[math_utils::Vector_Y];
        QuadrotorVelocityInertial(math_utils::Vector_Z) = DroneStateData.velocity[math_utils::Vector_Z];

        // step 1 calculate the current velocity
        PayloadVelocity = (1.0 - beta) * PreviousPayloadVelocity + beta * (PayloadPosition - PreviousPayloadPosition) / Ts ;
        // step 3 save current payload position as previous position
        PreviousPayloadVelocity = PayloadVelocity;       
        PreviousPayloadPosition = PayloadPosition;
        // step 4 get the rotation matrix of the quadrotor
        Quad_Drone(math_utils::Quat_w) = DroneStateData.attitude_q.w;
        Quad_Drone(math_utils::Quat_x) = DroneStateData.attitude_q.x;
        Quad_Drone(math_utils::Quat_y) = DroneStateData.attitude_q.y;
        Quad_Drone(math_utils::Quat_z) = DroneStateData.attitude_q.z;      
        // get the quadrotor rotation matrix          
        R_IB = QuaterionToRotationMatrix(Quad_Drone);
        Omega(math_utils::Vector_X) = DroneStateData.attitude_rate[math_utils::Vector_X];
        Omega(math_utils::Vector_Y) = DroneStateData.attitude_rate[math_utils::Vector_Y];
        Omega(math_utils::Vector_Z) = DroneStateData.attitude_rate[math_utils::Vector_Z];
        OmegaCross =  Hatmap(Omega);
        // step 6 get the inertial velocity of the payload
        PayloadVelocityInertial = R_IB *OmegaCross*PayloadPosition + R_IB*PayloadVelocity + QuadrotorVelocityInertial;
        PayloadPositionInertial = R_IB * PayloadPosition + QuadrotorPositionInertial;

        DroneStateData.header.stamp = ros::Time::now();
        // save the velocity in the drone state data
        DroneStateData.payload_vel[math_utils::Vector_X] = PayloadVelocityInertial(math_utils::Vector_X);
        DroneStateData.payload_vel[math_utils::Vector_Y] = PayloadVelocityInertial(math_utils::Vector_Y);
        DroneStateData.payload_vel[math_utils::Vector_Z] = PayloadVelocityInertial(math_utils::Vector_Z);

        DroneStateData.payload_pos[math_utils::Vector_X] = PayloadPositionInertial(math_utils::Vector_X);
        DroneStateData.payload_pos[math_utils::Vector_Y] = PayloadPositionInertial(math_utils::Vector_Y);
        DroneStateData.payload_pos[math_utils::Vector_Z] = PayloadPositionInertial(math_utils::Vector_Z);

        PubDroneState.publish(DroneStateData);
        DisplayVisionResults(PayloadPositionInertial, PayloadVelocityInertial, PayloadPosition,PayloadVelocity, Ts);
        rate.sleep();
    }

    return 0;
}