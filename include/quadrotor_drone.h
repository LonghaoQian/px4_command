#ifndef QUADROTOR_DRONE_COMMAND_H
#define QUADROTOR_DRONE_COMMAND_H
/*****************************************************

quadrotor_drone.h

Author: Longhao Qian
Date : 2020 08 01

A class containing all necessary parameters, functions, for a quadrotor drone

++++++++++++++++++++++++++++++++++++++++++++++++++++*/
#include <Eigen/Eigen>
#include <string>
#include <px4_command/DroneState.h>
#include <math_utils.h>
#include <px4_command_utils.h>
#include <iostream>
namespace experiment_drone {
    struct liftcurve{
        double motor_slope;
        double motor_intercept;        
    };

    struct IMUdata{
        Eigen::Vector4f Quad_Drone; // quadrotor attitude 
        Eigen::Vector3f AccBody;    // acc in body fixed frame
        Eigen::Vector3f AccInertial; // acc in inertial frame
        Eigen::Matrix3f R_Ij; // drone attitude
    };

    struct drone_command{
        Eigen::Vector3f accelSetpoint;     // acceleration setpoint based on translational control
        Eigen::Vector3f thrustSetpoint;    // thrust setpoint under tilt angle constraint
        Eigen::Vector3f throttleSetpoint;  // calculate the required throttle command based on the lift model
    };

    struct quadrotor_parameter{
        liftcurve liftmodel;
        float Quad_MASS;
        float tiltlimit;// maximum allowed tilt angle for drones (DEG)
        std::string uav_name;// uav name tag
    };

    class quadrotor_drone
    {
        public:
            quadrotor_drone();
            ~quadrotor_drone();
            std::string getUAVname();
            IMUdata getProcessedIMU();
            drone_command getDroneCommand();
            void loadparameter(const quadrotor_parameter& param);
            void updatestate(const px4_command::DroneState& _DroneState);
            px4_command::ControlOutput outputdronecommand(const Eigen::Vector3f& accel_command,  
                                                          const float& effective_mass,
                                                          const Eigen::Vector3f& u_l,
                                                          const Eigen::Vector3f& u_d);
            px4_command::ControlOutput outputdronecommand(const Eigen::Vector3f& desiredlift);                                                         
            void printf_param();
            void printf_state();
        private:
            quadrotor_parameter parameter;
            drone_command command;
            px4_command::ControlOutput output;
            IMUdata IMU;
            Eigen::Vector3f g_I; // gravity acc in inertial frame
    };

    quadrotor_drone::quadrotor_drone(){
            g_I = math_utils::GetGravitationalAcc();
    }

    quadrotor_drone::~quadrotor_drone(){

    }

    void quadrotor_drone::loadparameter(const quadrotor_parameter& param){
        parameter = param;
    }

    void quadrotor_drone::updatestate(const px4_command::DroneState& _DroneState){
        IMU.Quad_Drone(math_utils::Quat_w) = _DroneState.attitude_q.w;
        IMU.Quad_Drone(math_utils::Quat_x) = _DroneState.attitude_q.x;
        IMU.Quad_Drone(math_utils::Quat_y) = _DroneState.attitude_q.y;
        IMU.Quad_Drone(math_utils::Quat_z) = _DroneState.attitude_q.z;
        IMU.R_Ij =  QuaterionToRotationMatrix(IMU.Quad_Drone);
        // get the acceleration reading from IMU
        for( int i = 0; i < 3 ; i ++) {
            IMU.AccBody(i) = _DroneState.acceleration[i];
        }
        IMU.AccInertial = IMU.R_Ij * IMU.AccBody + g_I;// calculate true acc in inertial frame dot_vqj in TCST paper   
    }

    px4_command::ControlOutput quadrotor_drone::outputdronecommand(const Eigen::Vector3f& accelCommand,
                                                  const float& effective_mass,
                                                  const Eigen::Vector3f& u_l,
                                                  const Eigen::Vector3f& u_d){
        command.accelSetpoint =  accelCommand;
        command.thrustSetpoint  =  px4_command_utils::accelToThrust(command.accelSetpoint, 
                                                               effective_mass,
                                                               parameter.tiltlimit);
        // calculate the required throttle command
        command.throttleSetpoint = px4_command_utils::thrustToThrottleLinear(command.thrustSetpoint,
                                                                        parameter.liftmodel.motor_slope, 
                                                                        parameter.liftmodel.motor_intercept);

        for (int i=0; i<3; i++) {
            output.u_l[i] = u_l[i];
            output.u_d[i] = u_d[i];
            output.Thrust[i]   = command.thrustSetpoint(i);
            output.Throttle[i] = command.throttleSetpoint(i);
        }

        return output;
    }
    px4_command::ControlOutput quadrotor_drone::outputdronecommand(const Eigen::Vector3f& desiredlift) {

        command.thrustSetpoint  = px4_command_utils::ForceToThrust(desiredlift,parameter.tiltlimit);
        // calculate the required throttle command
        command.throttleSetpoint = px4_command_utils::thrustToThrottleLinear(command.thrustSetpoint,
                                                                             parameter.liftmodel.motor_slope, 
                                                                             parameter.liftmodel.motor_intercept);
        for (int i=0; i<3; i++) {
            output.u_l[i] = 0.0;
            output.u_d[i] = 0.0;
            output.Thrust[i]   = command.thrustSetpoint(i);
            output.Throttle[i] = command.throttleSetpoint(i);
        }
        return output;
    }
    std::string  quadrotor_drone::getUAVname(){
        return parameter.uav_name;
    }
    IMUdata quadrotor_drone::getProcessedIMU(){
        return IMU;
    }
    drone_command quadrotor_drone::getDroneCommand(){
        return command;
    }
    void quadrotor_drone::printf_param(){
        cout << parameter.uav_name << " parameter:  \n";
        cout <<"Quad_MASS : "<< parameter.Quad_MASS << " [kg] \n";
        cout << "Motor Curve Slop: " << parameter.liftmodel.motor_slope << " Motor Curve Intercept: "<<parameter.liftmodel.motor_intercept <<"\n";
        cout <<" Maximum tilt angle: "<< parameter.tiltlimit << " [DEG] \n";
    }
    void quadrotor_drone::printf_state(){
        // display total control force and throttle
        cout << parameter.uav_name << " control force [N] \n";
        cout << "Thrust Setpoint  [X Y Z] : " << 4*command.thrustSetpoint(math_utils::Vector_X) <<" [N] "
                                              << 4*command.thrustSetpoint(math_utils::Vector_Y) <<" [N] "
                                              << 4*command.thrustSetpoint(math_utils::Vector_Z) <<" [N] \n";
        cout << "Throttle Setpoint [X Y Z] : " << command.throttleSetpoint(math_utils::Vector_X) <<" [] " 
                                               << command.throttleSetpoint(math_utils::Vector_Y) << " [] " 
                                               << command.throttleSetpoint(math_utils::Vector_Z) << " [] \n";
        cout << "Accbody: [X Y Z] : " << IMU.AccBody(math_utils::Vector_X) <<" [m/s^2] "
                                      << IMU.AccBody(math_utils::Vector_Y) <<" [m/s^2] "
                                      << IMU.AccBody(math_utils::Vector_Z) <<" [m/s^2] \n";
        cout << "AccInertial: [X Y Z] : " << IMU.AccInertial(math_utils::Vector_X) <<" [m/s^2] "
                                      << IMU.AccInertial(math_utils::Vector_Y) <<" [m/s^2] "
                                      << IMU.AccInertial(math_utils::Vector_Z) <<" [m/s^2] \n";
        cout << " Quadrotor Attitude: []: \n";
        cout << IMU.R_Ij <<"\n"; 
    }
}

#endif