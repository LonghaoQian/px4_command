#ifndef POS_CONTROLLER_TIE_H
#define POS_CONTROLLER_TIE_H

/****************************

Author: Longhao Qian
Date : 2020 9 05

Robust control from TIE paper

Visual feedback from April Tag

***************************/

#include <Eigen/Eigen>
#include <math.h>
#include <command_to_mavros.h>
#include <px4_command_utils.h>
#include <math_utils.h>
#include <iostream>
#include <string>
#include <rectangular_trajectory.h>
#include <px4_command/DroneState.h>
#include <px4_command/TrajectoryPoint.h>
#include <px4_command/AttitudeReference.h>
#include <px4_command/ControlOutput.h>
#include <nav_msgs/Odometry.h>
#include <px4_command/AuxiliaryState_singleUAV.h>
#include <px4_command/SinglePayloadAction.h>
using std::string;
using std::iostream;


class pos_controller_TIE
{
    public:
        pos_controller_TIE(char drone_ID[20], ros::NodeHandle& main_handle)
        {
            // use drone ID to get the correct name for parameters
            uav_pref = "uav";
            uav_pref = uav_pref + drone_ID[0];
            main_handle.param<float>("Pos_tie/quadrotor_mass", Quad_MASS, 1.0);
            main_handle.param<float>("Pos_tie/payload_mass", Payload_Mass, 0.5);
            main_handle.param<float>("Pos_tie/cablelength", Cable_Length, 1.0);
            main_handle.param<double>("Pos_tie/motor_slope", motor_slope,0.3);
            main_handle.param<double>("Pos_tie/motor_intercept", motor_intercept, 0);

            T_tie.setZero();
            Lambda.setZero();
            main_handle.param<float>("Pos_tie/T_tie_xy", T_tie(0,0), 1.0);
            main_handle.param<float>("Pos_tie/T_tie_xy", T_tie(1,1), 1.0);
            main_handle.param<float>("Pos_tie/T_tie_z",  T_tie(2,2), 1.0);

            Lambda(0,0) = 1/T_tie(0,0);
            Lambda(1,1) = 1/T_tie(1,1);
            Lambda(2,2) = 1/T_tie(2,2);

            main_handle.param<bool>("Pos_tie/isPubAuxiliaryState", isPublishAuxiliarySate, false);

            main_handle.param<bool>("Pos_tie/isIntegrationOn", isIntegrationOn, false);

            Kp.setZero();
            Kv.setZero();
            Kpv.setZero();

            main_handle.param<float>("Pos_tie/Kp_xy", Kp(0,0), 1.0);
            main_handle.param<float>("Pos_tie/Kp_xy", Kp(1,1), 1.0);
            main_handle.param<float>("Pos_tie/Kp_z" , Kp(2,2), 2.0);

            main_handle.param<float>("Pos_tie/Kv_xy", Kv(0,0), 0.5);
            main_handle.param<float>("Pos_tie/Kv_xy", Kv(1,1), 0.5);
            main_handle.param<float>("Pos_tie/Kv_z",  Kv(2,2), 0.5);

            main_handle.param<float>("Pos_tie/Kpv_xy", Kpv(0,0), 0.0);
            main_handle.param<float>("Pos_tie/Kpv_xy", Kpv(1,1), 0.0);
            main_handle.param<float>("Pos_tie/Kpv_z",  Kpv(2,2), 0.0);

            main_handle.param<float>("Pos_tie/KL", KL, 0.0);

            main_handle.param<float>("Limitne/pxy_error_max", pos_error_max[0], 0.6);
            main_handle.param<float>("Limitne/pxy_error_max", pos_error_max[1], 0.6);
            main_handle.param<float>("Limit/pz_error_max" ,   pos_error_max[2], 1.0);
            main_handle.param<float>("Limit/vxy_error_max",   vel_error_max[0], 0.3);
            main_handle.param<float>("Limit/vxy_error_max",   vel_error_max[1], 0.3);
            main_handle.param<float>("Limit/vz_error_max" ,   vel_error_max[2], 1.0);

            Eigen::Vector3f  int_max_temp;

            main_handle.param<float>("Limit/pxy_int_max"  ,  int_max_temp(0), 0.5);
            main_handle.param<float>("Limit/pxy_int_max"  ,  int_max_temp(1), 0.5);
            main_handle.param<float>("Limit/pz_int_max"   ,  int_max_temp(2), 0.5);

            int_max = int_max_temp.norm();

            main_handle.param<float>("Limit/tilt_max"     ,     tilt_max, 20.0);
            main_handle.param<float>("Limit/int_start_error"  , int_start_error, 0.3);

            // read the drone geo fence data
            main_handle.param<float>("DroneGeoFence/x_min", geo_fence_x[0], -1.2);
            main_handle.param<float>("DroneGeoFence/x_max", geo_fence_x[1], 1.2);
            main_handle.param<float>("DroneGeoFence/y_min", geo_fence_y[0], -0.9);
            main_handle.param<float>("DroneGeoFence/y_max", geo_fence_y[1], 0.9);
            main_handle.param<float>("DroneGeoFence/z_min", geo_fence_z[0], 0.2);
            main_handle.param<float>("DroneGeoFence/z_max", geo_fence_z[1], 2);
            // read the trajectory information
            main_handle.param<int>("ActionMode/type", type, 0);

            switch (type) {
              case 0: {
                main_handle.param<float>("Circle_Trajectory/Center_x", Center(0), 0.0);
                main_handle.param<float>("Circle_Trajectory/Center_y", Center(1), 0.0);
                main_handle.param<float>("Circle_Trajectory/Center_z", Center(2), 1.0);
                main_handle.param<float>("Circle_Trajectory/radius", radius, 0.3);
                main_handle.param<float>("Circle_Trajectory/linear_vel", vd, 0.3);
                break;
              }
              case 1: {
                main_handle.param<float>("Rectangular_Trajectory/a_x",   rect_param.a_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/a_y",   rect_param.a_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_x", rect_param.v_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/vel_y", rect_param.v_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/h",     rect_param.h, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_x",     rect_param.center_x, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_y",     rect_param.center_y, 0.0);
                main_handle.param<float>("Rectangular_Trajectory/center_z",     rect_param.center_z, 0.0);
                rec_traj.LoadParameter(rect_param);

                break;
              }
              default: {
                main_handle.param<float>("Circle_Trajectory/Center_x", Center(0), 0.0);
                main_handle.param<float>("Circle_Trajectory/Center_y", Center(1), 0.0);
                main_handle.param<float>("Circle_Trajectory/Center_z", Center(2), 1.0);
                main_handle.param<float>("Circle_Trajectory/radius", radius, 0.3);
                main_handle.param<float>("Circle_Trajectory/linear_vel", vd, 0.3);
                break;
              }
            }


            // reset all the states
            u_l.setZero();
            u_d.setZero();
            u_p.setZero();
            u_s.setZero();
            r.setZero();
            v_p.setZero();
            UAV_attitude.setZero();
            integral.setZero();
            accel_sp.setZero();
            pos_error.setZero();
            vel_error.setZero();
            thrust_sp.setZero();
            throttle_sp.setZero();
            Identity.setIdentity();
            R_Ij.setIdentity();
            omega_j.setZero();
            dot_vq.setZero();
            accel_body.setZero();
            fL.setZero();
            L.setZero();
            B<<1.0,0.0,
               0.0,1.0,
               0.0,0.0;
            g_I<<0.0,
                 0.0,
                -9.81;
            unit_z<<0,
                    0,
                    1;
            W_hat.setZero();
            payload_position_vision.setZero();
            payload_velocity_vision.setZero();
            payload_position_mocap.setZero();
            payload_velocity_mocap.setZero();
            Cable_Length_sq = Cable_Length*Cable_Length;
            TotalLiftingMass = Quad_MASS + Payload_Mass;

            isEmergency = false;
            isperformAction = false;
            // initialize subscriber:
            serverAction = main_handle.advertiseService("/" + uav_pref +"/px4_command/action", &pos_controller_TIE::ResponseToActionCall, this);
            if (isPublishAuxiliarySate)  {
               pubAuxiliaryState   = main_handle.advertise<px4_command::AuxiliaryState_singleUAV> ("/" + uav_pref + "/px4_command/auxiliarystate_singleUAV", 1000);
            }
            // initialize
        }
        void printf_param();
        void printf_result();
        bool emergency_switch();
        px4_command::ControlOutput pos_controller(const px4_command::DroneState& _DroneState,
                                                  const px4_command::TrajectoryPoint& _Reference_State,
                                                  float dt);
        px4_command::ControlOutput _ControlOutput;
    private:
        // ------------ private functions-----------------//
        void SendAuxiliaryState();
        bool ResponseToActionCall(px4_command::SinglePayloadAction::Request& req, px4_command::SinglePayloadAction::Response& res);
        bool DroneGeoFenceCheck();
        //------------- private variables ---------------//
        ros::Publisher       pubAuxiliaryState;
        ros::ServiceClient   clientSendParameter;
        ros::ServiceServer   serverAction;
        bool isPublishAuxiliarySate;
        bool isIntegrationOn;
        bool isEmergency;
        bool isperformAction;
        // action info
        int type;
        Eigen::Vector3f Center;
        float radius;
        float vd;
        trajectory::Reference_Path rect_path;
        trajectory::Rectangular_Trajectory_Parameter rect_param;
        trajectory::Rectangular_Trajectory rec_traj;
        // geo geo_fence
        Eigen::Vector2f geo_fence_x;
        Eigen::Vector2f geo_fence_y;
        Eigen::Vector2f geo_fence_z;
        //quadrotor and payload parameter
        string mode;
        string uav_pref;
        float Quad_MASS;
        float Payload_Mass;
        float Cable_Length;
        float Cable_Length_sq;
        float TotalLiftingMass;
        double motor_slope;
        double motor_intercept;
        Eigen::Vector3f g_I;
        Eigen::Vector3f unit_z;
        // payload states
        Eigen::Vector3f L;    // cable vector
        Eigen::Vector3f L_dot;// cable velocity
        Eigen::Vector2f r;
        Eigen::Vector2f v_p;
        Eigen::Matrix<float, 3,2> B;
        Eigen::Matrix3f R_Ij;
        Eigen::Vector3f dot_vq;
        Eigen::Vector3f accel_body;

        //controller parameters

        Eigen::Matrix3f Kp;
        Eigen::Matrix3f Kv;
        Eigen::Matrix3f T_tie;
        Eigen::Matrix3f Lambda;
        Eigen::Matrix3f Kpv;
        Eigen::Matrix3f Identity;
        float KL;

        //control output limit
        Eigen::Vector3f reference_position;
        Eigen::Vector3f UAV_position;
        Eigen::Vector3f UAV_velocity;
        Eigen::Vector4f UAV_attitude;
        Eigen::Vector3f omega_j;
        Eigen::Vector3f payload_position_vision;
        Eigen::Vector3f payload_velocity_vision;
        Eigen::Vector3f payload_position_mocap;
        Eigen::Vector3f payload_velocity_mocap;
        Eigen::Vector3f pos_error;
        Eigen::Vector3f vel_error;
        Eigen::Vector3f pos_error_max;
        Eigen::Vector3f vel_error_max;
        float int_max;
        float tilt_max;
        float int_start_error;

        //control states
        Eigen::Vector3f u_l,u_d,u_p,u_s; //u_l for nominal contorol(PD), u_d for ude control(disturbance estimator)
        Eigen::Vector3f integral;
        Eigen::Vector3d accel_sp;
        Eigen::Vector3d thrust_sp;
        Eigen::Vector3d throttle_sp;
        Eigen::Vector3f fL;
        Eigen::Vector3f W_hat;
        bool isintegrationoverlimit{false};
};

px4_command::ControlOutput pos_controller_TIE::pos_controller(
    const px4_command::DroneState& _DroneState,
    const px4_command::TrajectoryPoint& _Reference_State,
    float dt)
{
    mode = _DroneState.mode;
    // get quadrotor position and velocity
    for (int i=0; i<3; i++) {
        reference_position(i) = _Reference_State.position_ref[i];
        UAV_position(i) = _DroneState.position[i];
        UAV_velocity(i) = _DroneState.velocity[i];
        payload_position_mocap(i) = _DroneState.payload_pos[i];
        payload_velocity_mocap(i) = _DroneState.payload_vel[i];
        omega_j(i) = _DroneState.attitude_rate[i]; // verify the attitude is in the body fixed frame
    }

    // get the quadrotor rotation matrix
    UAV_attitude(math_utils::Quat_w) = _DroneState.attitude_q.w;
    UAV_attitude(math_utils::Quat_x) = _DroneState.attitude_q.x;
    UAV_attitude(math_utils::Quat_y) = _DroneState.attitude_q.y;
    UAV_attitude(math_utils::Quat_z) = _DroneState.attitude_q.z;
    R_Ij =  QuaterionToRotationMatrix(UAV_attitude);

    // get the acceleration reading from IMU
    for( int i = 0; i < 3 ; i ++) {
        accel_body(i) = _DroneState.acceleration[i];
    }
    dot_vq = R_Ij * (accel_body) + g_I;// calculate true acc in inertial frame dot_vqj 

    // if not, switch back to mocap signal
    L     = - UAV_position + payload_position_mocap;
    L_dot = - UAV_velocity + payload_velocity_mocap;

    r(math_utils::Vector_X) = L(math_utils::Vector_X);
    r(math_utils::Vector_Y) = L(math_utils::Vector_Y);
    v_p(math_utils::Vector_X) = L_dot(math_utils::Vector_X);
    v_p(math_utils::Vector_Y) = L_dot(math_utils::Vector_Y);

    // determine the geo_fence
    if(!DroneGeoFenceCheck()){// if out of bound, return to normal mode
        isperformAction = false;
    }
    // calculate quadrotor position and velocity error:
    if(isperformAction){
      switch (type) {
        case 0:{
          Eigen::Vector3f e1 = Center - UAV_position;
          Eigen::Vector3f n;
          float e_norm = e1.norm();
          n.setZero();
          if(e_norm<0.1){// for too a very small position error, set the norm and unit vector to a fixed number
            n(0) = 1;
            n(1) = 0;
            n(2) = 0;
            e_norm = 0.1;
          }else{
            n = e1.normalized();
          }
          Eigen::Vector3f vd_direction;
          vd_direction = unit_z.cross(n);
          vd_direction.normalize();// normalize itself

          vel_error = vd*vd_direction - UAV_velocity;
          pos_error =  (1-radius/e_norm )*e1;

          break;
        }
        case 1:{
            rect_path = rec_traj.UpdatePosition(UAV_position);
            vel_error = rect_path.vd*rect_path.n - UAV_velocity;
            pos_error = (Identity - rect_path.n*rect_path.n.transpose())*(rect_path.P-UAV_position);
            break;
        }

        default:{

          break;
        }
      }
    }else{
      pos_error = reference_position - UAV_position;
      vel_error = - UAV_velocity; // position stabilization
    }

    float sq_r = r(0)*r(0) + r(1)*r(1);
    // update B matrix:
    if (Cable_Length_sq - sq_r>0.01) {
        B(2,0) =  r(0)/sqrt((Cable_Length_sq - sq_r));
        B(2,1) =  r(1)/sqrt((Cable_Length_sq - sq_r));
    }else {
        B(2,0) = 0.1;
        B(2,1) = 0.1;
    }

    u_p.setZero();
    u_p = Kpv * u_p;
    u_s = Kp * pos_error + Kv * vel_error;
    u_s = constrain_vector(u_s, 0.5);// constraint the error

    // calculate the integral term of the UDE 
    if((_DroneState.mode == "OFFBOARD") && isIntegrationOn) {
        if(u_s.norm() < int_start_error) {
            // check whether if the integral is out of bound
            if(integral.norm() > int_max)  {
                // if the norm is over limit, stop the integration
                isintegrationoverlimit = true;
            } else {
                isintegrationoverlimit = false;
                integral += - u_s * dt;
            }
        }
    } else {
        integral.setZero();
    }
    // calcualte total control force
    u_l = u_s + u_p;

    u_d = Lambda * (Payload_Mass*(Identity + Kpv)* B * (v_p + KL*r) + Quad_MASS*UAV_velocity  + integral)/(TotalLiftingMass);
    W_hat = TotalLiftingMass * u_d;
    accel_sp[0] = u_l[0] - u_d[0];
    accel_sp[1] = u_l[1] - u_d[1];
    accel_sp[2] = u_l[2] - u_d[2] + 9.81; // + 9.81 for counteracting gravity
    fL = TotalLiftingMass * accel_sp.cast <float> ();
    thrust_sp   = px4_command_utils::accelToThrust(accel_sp, TotalLiftingMass, tilt_max);
    throttle_sp = px4_command_utils::thrustToThrottleLinear(thrust_sp, motor_slope, motor_intercept);
    // publish auxiliary state
    if (isPublishAuxiliarySate) {
        SendAuxiliaryState();
    }
    for (int i=0; i<3; i++)
    {
        _ControlOutput.u_l[i] = u_l[i];
        _ControlOutput.u_d[i] = u_d[i];
        _ControlOutput.Thrust[i] = thrust_sp[i];
        _ControlOutput.Throttle[i] = throttle_sp[i];
    }

    return _ControlOutput;

}

bool pos_controller_TIE::emergency_switch()
{
    /* 1. check whether the total length of the string is within a safe range */
    if (L.norm()>Cable_Length*1.2)
    {
        isEmergency = true;
    }
    /* 2. check if the r_j is too large */

    if (r.norm()>Cable_Length*sin(50/57.3))
    {
        isEmergency = true;
    }

    return isEmergency;
}

void pos_controller_TIE::SendAuxiliaryState()
{
    px4_command::AuxiliaryState_singleUAV msg;
    msg.header.stamp = ros::Time::now();
    msg.W_x  = W_hat(math_utils::Vector_X);
    msg.W_y  = W_hat(math_utils::Vector_Y);
    msg.W_z  = W_hat(math_utils::Vector_Z);

    msg.q_0 = UAV_attitude(0);
    msg.q_1 = UAV_attitude(1);
    msg.q_2 = UAV_attitude(2);
    msg.q_3 = UAV_attitude(3);

    msg.r_x = r(math_utils::Vector_X);
    msg.r_y = r(math_utils::Vector_Y);

    msg.v_x = v_p(math_utils::Vector_X);
    msg.v_y = v_p(math_utils::Vector_Y);

    msg.pos_error_x = pos_error(math_utils::Vector_X);
    msg.pos_error_y = pos_error(math_utils::Vector_Y);
    msg.pos_error_z = pos_error(math_utils::Vector_Z);

    msg.vel_error_x = vel_error(math_utils::Vector_X);
    msg.vel_error_y = vel_error(math_utils::Vector_Y);
    msg.vel_error_z = vel_error(math_utils::Vector_Z);

    msg.Lm_x = payload_position_mocap(math_utils::Vector_X);
    msg.Lm_y = payload_position_mocap(math_utils::Vector_Y);
    msg.Lm_z = payload_position_mocap(math_utils::Vector_Z);

    msg.Vpm_x = payload_velocity_mocap(math_utils::Vector_X);
    msg.Vpm_y = payload_velocity_mocap(math_utils::Vector_Y);
    msg.Vpm_z = payload_velocity_mocap(math_utils::Vector_Z);

    msg.fL_x = fL(0);
    msg.fL_y = fL(1);
    msg.fL_z = fL(2);

    msg.acc_x  = dot_vq(0);
    msg.acc_y  = dot_vq(1);
    msg.acc_z  = dot_vq(2);

    pubAuxiliaryState.publish(msg);
}

bool pos_controller_TIE::ResponseToActionCall(px4_command::SinglePayloadAction::Request& req, px4_command::SinglePayloadAction::Response& res){

    if(DroneGeoFenceCheck()&&req.perform_action){
        isperformAction = req.perform_action;
    }else{
        isperformAction = false;
    }
    res.status_ok = isperformAction;
    res.trajectory_type = type;
    return true;
}

bool pos_controller_TIE::DroneGeoFenceCheck(){
    if (UAV_position(0) < geo_fence_x[0] || UAV_position(0) > geo_fence_x[1] ||
        UAV_position(1) < geo_fence_y[0] || UAV_position(1) > geo_fence_y[1] ||
        UAV_position(2) < geo_fence_z[0] || UAV_position(2) > geo_fence_z[1]) {
        return false;
    } else {
        return true;
    }
}


void pos_controller_TIE::printf_result()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>  Single UAV Payload Position Controller (TIE) <<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout<<setprecision(2);
    cout << "The autopilot mode is : " << mode << endl;
    cout << "Pos Error [X Y Z] : " << pos_error(0) << " [m] " << pos_error(1) << " [m] " << pos_error(2) << " [m] " << endl;
    cout << "Vel Error [X Y Z] : " << vel_error(0) << " [m/s] " << vel_error(1) << " [m/s] " << vel_error(2) << " [m/s] " << endl;
    cout << "u_l [X Y Z] : " << u_l[0] << " [N] "<< u_l[1]<<" [N] "<<u_l[2]<<" [N] "<<endl;
    cout << "u_d [X Y Z] : " << u_d[0] << " [N] "<< u_d[1]<<" [N] "<<u_d[2]<<" [N] "<<endl;
    if (isintegrationoverlimit) {
        cout << "Integration over limit !" << endl;
    }
    cout << "int [X Y Z] : " << integral[0] << " [N] "<< integral[1]<<" [N] "<<integral[2]<<" [N] "<<endl;
    cout << "r [X Y] : " << r(0) << " [m] " << r(1) << "[m] " <<endl;
    cout << "v_p [X Y] : " << v_p(0) << " [m/s] " << v_p(1) <<  " [m/s] " << endl;
    cout << "fL [X Y Z] : " << fL(0) << " [N] " << fL(1) << " [N] " << fL(2) << " [N] " <<endl;
    cout << "B matrix is : " << endl;
    cout << B << endl;
    cout << "W_hat [X Y Z] : " << W_hat(math_utils::Vector_X) << " [N], " << W_hat(math_utils::Vector_Y) << " [N], " << W_hat(math_utils::Vector_Z) << " [N]. \n"; 

    if(isperformAction){
      cout << "---Perfroming Action...--- \n";
    }else{
      cout << "---Not Performing Action, Normal Flight.--- \n";
    }

    cout<< "payload velocity from mocap is [X Y Z] : " << payload_velocity_mocap(math_utils::Vector_X) << " [m/s] "
                                                  << payload_velocity_mocap(math_utils::Vector_Y) << " [m/s] "
                                                  << payload_velocity_mocap(math_utils::Vector_Z) << " [m/s] " << endl;

    cout<< "payload position from mocap is [X Y Z] : " << payload_position_mocap(math_utils::Vector_X) << " [m] "
                                                  << payload_position_mocap(math_utils::Vector_Y) << " [m] "
                                                  << payload_position_mocap(math_utils::Vector_Z) << " [m] " <<endl;


     switch (type) {
      case 0:
      {
        break;
      }
      case 1:
      {
        rec_traj.printf_result();
        break;
      }
      default:
      {
        break;
      }
    }

    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<endl;
}

// 【打印参数函数】
void pos_controller_TIE::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Payload control method in TIE paper (Parameter)  <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << uav_pref << " is used for single payload stabilization" << endl;
    cout <<"Vehicle Paramter: " <<endl;
    cout <<"Quad_MASS : "   << Quad_MASS << " [kg] "<<endl;
    cout <<"Payload_MASS : "<< Payload_Mass << " [kg] "<< endl;
    cout <<"Cable_Length : "<< Cable_Length << " [m] "<< endl;
    cout << "motor_slope: " << motor_slope << " motor_intercept: " << motor_intercept << endl;

    cout <<"Kp_x : "<< Kp(0,0) << endl;
    cout <<"Kp_y : "<< Kp(1,1) << endl;
    cout <<"Kp_z : "<< Kp(2,2) << endl;

    cout <<"T_tie_x : "<< T_tie(0,0) << endl;
    cout <<"T_tie_y : "<< T_tie(1,1) << endl;
    cout <<"T_tie_z : "<< T_tie(2,2) << endl;

    cout <<"Kv_x : "<< Kv(0,0) << endl;
    cout <<"Kv_y : "<< Kv(1,1) << endl;
    cout <<"Kv_z : "<< Kv(2,2) << endl;

    cout <<"Kpv_x : "<< Kpv(0,0)<<endl;
    cout <<"Kpv_y : "<< Kpv(1,1)<<endl;
    cout <<"Kpv_z : "<< Kpv(2,2)<<endl;

    cout << "KL : " << KL << endl;

    cout <<"Control output limit:  " <<endl;
    cout <<"vxy_error_max : "<< vel_error_max[0] << endl;
    cout <<"vz_error_max :  "<< vel_error_max[2] << endl;
    cout <<"int_max : "<< int_max << endl;
    cout <<"tilt_max : "<< tilt_max << endl;
    cout <<"int_start_error : "<< int_start_error << endl;
    cout << "Lambda : " <<endl;
    cout << Lambda << endl;

    if (isPublishAuxiliarySate) {
        cout << "Auxiliary state is going to be published! " <<endl;
    } else {
        cout << "No Auxiliary state published! " <<endl;
    }

    cout<<"Geofence for action is: \n";
    cout<<"DroneGeoFence X min: " <<geo_fence_x[0] << " (m), max: " << geo_fence_x[1] << " (m) \n";
    cout<<"DroneGeoFence Y min: " <<geo_fence_y[0] << " (m), max: " << geo_fence_y[1] << " (m) \n"; 
    cout<<"DroneGeoFence Z min: " <<geo_fence_z[0] << " (m), max: " << geo_fence_z[1] << " (m) \n"; 

    switch (type) {
      case 0:
      {
        cout<<"circle trajectory is loaded"<<endl;
        cout<<"center of circle is : X: "<< Center(0) << "m, Y: " << Center(1) << "m, Z: "<<Center(2) << "m" <<endl;
        cout<<"radius is: " << radius << " m"<<endl;
        cout<<"reference velocity is: "<< vd << " m"<<endl;
        break;
      }
      case 1:
      {
        rec_traj.printf_param();
        break;
      }
      default:
      {
        cout<<"circle trajectory is loaded"<<endl;
        cout<<"center of circle is : X: "<< Center(0) << "m, Y: " << Center(1) << "m, Z: "<<Center(2) << "m" <<endl;
        cout<<"radius is: " << radius << " m"<<endl;
        cout<<"reference velocity is: "<< vd << " m"<<endl;
        break;
      }
    }

    cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" <<endl;
}

#endif
