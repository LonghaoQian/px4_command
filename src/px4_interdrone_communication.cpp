/***************************************************************************************************************************
 * px4_interdrone_communication.cpp
 *
 * Author: Longhao Qian
 *
 * Update Time: 2019.12.02
 * Get the control force and quadrotor relative state for robust cooperative payload control 
 * if the drone is uav0 
 * subscribe the /uav#/px4_command/control_output (f_Lj)
 * subscribe  
 * publish Delta_T and Delta_R 
 * if the drone is not uav0 
 * publish f_Lj and delta_j
 * subscribe Delta_T and Delta_R from uav0 
***************************************************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math_utils.h>
#include <px4_command/DroneState.h>
#include <px4_command/AddonForce.h>
#include <px4_command/FleetStatus.h>
#include <px4_command/ControlCommand.h>
#include <px4_command_utils.h>
#include <command_to_mavros_multidrone.h>
using std::vector;
using std::string;

// messages:
static px4_command::AddonForce  _AddonForce;// addonforce to be published
static px4_command::DroneState  _DroneState;// dronestate from uav0
static px4_command::ControlCommand Command_Now;
//estimation:
static Eigen::Vector3f Delta_T_I;
static Eigen::Vector3f Delta_R_I;
static Eigen::Vector3f Delta_T;
static Eigen::Vector3f Delta_R;
static Eigen::Vector3f Delta_rt;
static Eigen::Vector3f Delta_pt;
static Eigen::Matrix3f Delta_sq;
// states 
<<<<<<< HEAD
Eigen::Matrix<float,2,3> r_sq;
Eigen::Matrix<float,2,3> v_sq;
Eigen::Matrix<float,2,3> rd_sq;
Eigen::Matrix3f f_L_sq;// 
float cur_time;
Eigen::Matrix3f R_IP;
Eigen::Matrix3f R_PI;
Eigen::Vector3f v_p;
Eigen::Vector3f omega_p;
Eigen::Matrix3f omega_p_cross;
=======
static Eigen::Matrix<float,2,3> r_sq;
static Eigen::Matrix<float,2,3> rd_sq;
static Eigen::Matrix<float,2,3> v_sq;
static Eigen::Matrix3f f_L_sq;// 
static float cur_time;
static Eigen::Matrix3f R_IP;
static Eigen::Matrix3f R_PI;
static Eigen::Vector3f v_p;
static Eigen::Vector3f omega_p;
static Eigen::Matrix3f omega_p_cross;
>>>>>>> TCST_complete
// auxiliary variables:
static Eigen::Vector3f FT;
static Eigen::Vector3f FR;
static Eigen::Vector3f FR2;
static Eigen::Vector3f BT;
static Eigen::Vector3f DT;
static Eigen::Vector3f DR;
static Eigen::Matrix<float,3,2> B_j;
// parameters:
<<<<<<< HEAD
int   num_of_drones;
float payload_mass;
float M_q;
Eigen::Vector3f g_I;
Eigen::Matrix3f t_sq;
Eigen::Matrix3f A;
Eigen::Matrix3f J_q;
Eigen::Matrix3f J_p;
Eigen::Matrix3f D;
Eigen::Vector3f quadrotor_mass;
Eigen::Vector3f a_j_sq;
Eigen::Matrix3f lambda_T;
Eigen::Matrix3f lambda_R;
Eigen::Vector3f cablelength;
Eigen::Vector3f cablelength_squared;
Eigen::Matrix<float 3, Dynamic> E_j;
// cross feeding terms
Eigen::Vector3f F1,F2,R1,R2, Zeta, Eta;
=======
static int   num_of_drones;
static float payload_mass;
static float M_q;
static Eigen::Vector3f g_I;
static Eigen::Matrix3f t_sq;
static Eigen::Matrix3f A;
static Eigen::Matrix3f J_q;
static Eigen::Matrix3f J_p;
static Eigen::Matrix3f D;
static Eigen::Vector3f quadrotor_mass;
static Eigen::Vector3f a_j_sq;
static Eigen::Matrix3f lambda_T;
static Eigen::Matrix3f lambda_R;
static Eigen::Vector3f cablelength;
static Eigen::Vector3f cablelength_squared;
static Eigen::Vector3f R1,R2;
static Eigen::Matrix<float,3, Eigen::Dynamic> E_j;
>>>>>>> TCST_complete
void GetCommand(const px4_command::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;
}

void GetDroneState(const px4_command::DroneState::ConstPtr& msg) {
    _DroneState = *msg;
    _DroneState.time_from_start = cur_time;
}

void GetUAV0Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav0_status;
    uav0_status = *msg;
    r_sq(0,0) = uav0_status.r_jx;
    r_sq(1,0) = uav0_status.r_jy;
    v_sq(0,0) = uav0_status.v_jx;
    v_sq(1,0) = uav0_status.v_jy;
    f_L_sq(0,0) = uav0_status.f_Ljx;
    f_L_sq(1,0) = uav0_status.f_Ljy;
    f_L_sq(2,0) = uav0_status.f_Ljz;
    Delta_sq(0,0) = uav0_status.delta_jx;
    Delta_sq(1,0) = uav0_status.delta_jy;
    Delta_sq(2,0) = uav0_status.delta_jz;
    rd_sq(0,0) = uav0_status.rd_jx;
    rd_sq(1,0) = uav0_status.rd_jy;
}
void GetUAV1Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav1_status;
    uav1_status = *msg;
    r_sq(0,1) = uav1_status.r_jx;
    r_sq(1,1) = uav1_status.r_jy;
    v_sq(0,1) = uav1_status.v_jx;
    v_sq(1,1) = uav1_status.v_jy;
    f_L_sq(0,1) = uav1_status.f_Ljx;
    f_L_sq(1,1) = uav1_status.f_Ljy;
    f_L_sq(2,1) = uav1_status.f_Ljz;
    Delta_sq(0,1) = uav1_status.delta_jx;
    Delta_sq(1,1) = uav1_status.delta_jy;
    Delta_sq(2,1) = uav1_status.delta_jz;
    rd_sq(0,1) = uav1_status.rd_jx;
    rd_sq(1,1) = uav1_status.rd_jy;
}
void GetUAV2Status(const px4_command::FleetStatus::ConstPtr& msg){
    px4_command::FleetStatus uav2_status;
    uav2_status = *msg;
    r_sq(0,2) = uav2_status.r_jx;
    r_sq(1,2) = uav2_status.r_jy;
    v_sq(0,2) = uav2_status.v_jx;
    v_sq(1,2) = uav2_status.v_jy;
    f_L_sq(0,2) = uav2_status.f_Ljx;
    f_L_sq(1,2) = uav2_status.f_Ljy;
    f_L_sq(2,2) = uav2_status.f_Ljz;
    Delta_sq(0,2) = uav2_status.delta_jx;
    Delta_sq(1,2) = uav2_status.delta_jy;
    Delta_sq(2,2) = uav2_status.delta_jz;
    rd_sq(0,2) = uav2_status.rd_jx;
    rd_sq(1,2) = uav2_status.rd_jy;
}

void PrintEstimation(){
    cout <<">>>>>>>>  TCST 2019 Disturbance Estimator <<<<<<<" <<endl;
    cout.setf(ios::fixed);//固定的浮点显示
    cout.setf(ios::left);//左对齐
    cout.setf(ios::showpoint);// 强制显示小数点
    cout.setf(ios::showpos);// 强制显示符号
    cout<<setprecision(3);// number of decimals 
    cout << "Current command mode is: " << _DroneState.mode << endl;
    if(_DroneState.mode == "OFFBOARD") {
        if (Command_Now.Mode == command_to_mavros_multidrone::Payload_Stabilization) {
            // display fleet info first
            for (int i = 0; i < num_of_drones ; i ++) {
                cout << ">>>>>>> UAV "<< i << " info <<<<<<<" << endl;
                cout << "Delta_"<< i <<" [X Y Z] : " << Delta_sq(0,i) << " [N] "<< Delta_sq(1,i) <<" [N] "<< Delta_sq(2,i)<<" [N] "<<endl;
                cout << "F_L_" << i << " [X Y Z] : " << f_L_sq(0,i) <<" [N] "<<f_L_sq(1,i) <<" [N] "<< f_L_sq(2,i)<<" [N] "<<endl;
                cout << "r_" << i << " [X Y] : "<< r_sq(0,i) << " [m] " << r_sq(1,i) << " [m] "    << endl;
                cout << "v_" << i << " [X Y] : "<< v_sq(0,i) << " [m/s] " << v_sq(1,i) << " [m/s] "<< endl;
                cout << "rd_" << i << " [X Y] : " << rd_sq(0,i) << " [m] " << rd_sq(1,i) << " [m] " << endl;
            }
            // then display estimation results
            cout << ">>>>>>> IN PAYLOAD STABILIZATION MODE, CALCULATING ESTIMATION <<<<<<<" << endl;
            cout << "Delta_T [X Y Z] : " << Delta_T(0) << " [N] "<< Delta_T(1)<<" [N] "<<Delta_T(2)<<" [N] "<<endl;
            cout << "Delta_pt [X Y Z] : " << Delta_pt(0) << " [N] "<< Delta_pt(1)<<" [N] "<<Delta_pt(2)<<" [N] "<<endl;
            cout << "Delta_R [X Y Z] : " << Delta_R(0) << " [N] "<< Delta_R(1)<<" [N] "<<Delta_R(2)<<" [N] "<<endl;
            cout << "Delta_rt [X Y Z] : " << Delta_rt(0) << " [N] "<< Delta_rt(1)<<" [N] "<<Delta_rt(2)<<" [N] "<<endl;
            cout << "R1 [X Y Z] : " << R1(0) << " [N] " << R1(1) << " [N] " << R1(2) << " [N] " <<endl;
            cout << "R2 [X Y Z] : " << R2(0) << " [N] " << R2(1) << " [N] " << R2(2) << " [N] " <<endl;
        } else {
            cout << ">>>>>>> NOT IN PAYLOAD STABILIZATION MODE, ESTIMATION PAUSED <<<<<<< \n";
        }
    } else {
        cout << ">>>>>>> NOT IN OFFBOARD MODE <<<<<<< \n";
    }
}

void DisplayParameters() {
    cout <<">>>>>>>> Parameter For UDE Estimator  <<<<<<<<<" <<endl;
    cout <<"Estimator Time Constants: "<< endl;
    cout <<"lambda_T: "<< lambda_T(0,0) << " " << lambda_T(1,1) << " " << lambda_T(2,2) << " " << endl;
    cout <<"lambda_R: "<< lambda_R(0,0) << " " << lambda_R(1,1) << " " << lambda_R(2,2) << " " << endl;
    cout <<"Payload Mass: " << payload_mass << " [kg]" <<endl;
    cout << num_of_drones << " drones are used ! "<< endl;
    for (int i = 0; i< num_of_drones ; i ++) {
        cout << ">>>>>>> ---- <<<<<<<<" <<endl;
        cout <<"Quadrotor " << i << " mass: " << quadrotor_mass(i) << " [kg] "<<  endl;
        cout << "a_" << i << " = " << a_j_sq(i) << endl;
        cout << "Tether Point [X Y Z]: "<< t_sq(0,i) << " [m] " << t_sq(1,i) << " [m] " << t_sq(2,i) << " [m] " << endl;
        cout << "cablelength_" << i << " = " << cablelength(i) << " [m] " <<endl;
        cout << "E_j matrix is : " << endl;
        cout << E_j.block(0,i*3,3,3) <<endl;
    }
    cout << "Total Quadrotor Mass: " << M_q << " [kg] " << endl;
    cout << "J_q : " << endl;
    cout <<  J_q << endl;
    cout << "J_p : " << endl;
    cout <<  J_p << endl;
    cout << "A : " << endl;
    cout << A << endl;
}

void ResetStates() {
    Delta_T_I.setZero();
    Delta_R_I.setZero();
    Delta_T.setZero();
    Delta_R.setZero();
    Delta_pt.setZero();
    Delta_rt.setZero();
}

int main(int argc, 
         char **argv) 
{
    ros::init(argc, argv, "px4_interdrone_communication");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    // initialize all parameters: 
    double kL = 0;
    lambda_T.setZero();
    lambda_R.setZero();
    A.setZero();
    J_q.setZero();
    a_j_sq.setZero();
    g_I << 0.0,
           0.0,
          -9.81;// ENU coordinates are used !!
    /* an estimation of payload moment of inertia. accurate moment of inertia is unecessary because more payload will be added during flight test*/
    J_p << 0.1, 0.0, 0.0,
           0.0, 0.1, 0.0,
           0.0, 0.0, 0.1;
    float kL = 0;
    // loading all parameters into the estimator:
    nh.param<int>   ("Pos_GNC/num_drone",num_of_drones,1);
    nh.param<float> ("Payload/mass", payload_mass, 1.0);

    nh.param<float> ("Pos_GNC/lambda_Txy", lambda_T(0,0),0.2);
    nh.param<float> ("Pos_GNC/lambda_Txy", lambda_T(1,1),0.2);
    nh.param<float> ("Pos_GNC/lambda_Tz", lambda_T(2,2),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rxy", lambda_R(0,0),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rxy", lambda_R(1,1),0.2);
    nh.param<float> ("Pos_GNC/lambda_Rz", lambda_R(2,2),0.2);
<<<<<<< HEAD
    nh.param<float> ("Pos_GNC/ kL", kL, 0.1);
    nh.param<float> ("Payload/mass", payload_mass, 1.0);
    Eigen::Vector3f temp_t_j;
    M_q = 0.0;// total mass of all quadrotorsa_j_sq
    E_j.resize(3,num_of_drones*3);
    E_j.setZero();
=======
    nh.param<float> ("Pos_GNC/kL", kL, 0.1);
    
    // temp variables
    Eigen::Vector3f temp_t_j;
    M_q = 0.0;// total mass of all quadrotorsa_j_sq
    float e_n = 1.0;
    E_j.resize(3,num_of_drones*3);
>>>>>>> TCST_complete
    for (int i = 0; i < num_of_drones ; i ++) {
        temp_t_j.setZero();
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_x", temp_t_j(0), 0.5);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_y", temp_t_j(1), 0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/TetherOffset_z", temp_t_j(2), 0); 
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/PayloadSharingPortion", a_j_sq(i), 0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/mass", quadrotor_mass(i), 1.0);
        nh.param<float>("uav" + to_string(i) + "_Pos_GNC/cablelength", cablelength(i), 1.0);
        cablelength_squared(i) = cablelength(i) * cablelength(i);
        D += a_j_sq(i) * Hatmap(temp_t_j)* Hatmap(temp_t_j);
        t_sq.col(i)  = temp_t_j;
        M_q += quadrotor_mass(i);
        A += quadrotor_mass(i) * Hatmap(temp_t_j);
        J_q += - quadrotor_mass(i) * Hatmap(temp_t_j) * Hatmap(temp_t_j);
<<<<<<< HEAD
        // TO DO: calculate Ej
=======
        e_n *= temp_t_j.norm();
    }
    // for 2 drone case, E_j is special

    for (int i = 0;  i< num_of_drones; i++ ) {
        if (num_of_drones < 3) { 
            E_j.block(0,i*3,3,3) = - Hatmap(t_sq.col(i)) / e_n;
        } else {
            E_j.block(0,i*3,3,3) = Hatmap(t_sq.col(i)) * D.inverse();
        }
    }
>>>>>>> TCST_complete

    }
    for (int i = 0, i< num_of_drones; i++ ) {
        E_j.segment(0,i*3,3,3) = Hatmap(t_sq.col(i)) * D.inverse();
    }
    // form the communication channels
    ros::Subscriber subUAV0status = nh.subscribe<px4_command::FleetStatus>("/uav0/px4_command/fleetstatus", 100, GetUAV0Status);
    ros::Subscriber subUAV1status = nh.subscribe<px4_command::FleetStatus>("/uav1/px4_command/fleetstatus", 100, GetUAV1Status);
    ros::Subscriber subUAV2status = nh.subscribe<px4_command::FleetStatus>("/uav2/px4_command/fleetstatus", 100, GetUAV2Status);
    ros::Publisher  pubAddonForce = nh.advertise<px4_command::AddonForce> ("/uav0/px4_command/addonforce", 1000);
    ros::Subscriber subdronestate = nh.subscribe<px4_command::DroneState>("/uav0/px4_command/drone_state", 100, GetDroneState);
    ros::Subscriber subCommand    = nh.subscribe<px4_command::ControlCommand>("/uav0/px4_command/control_command", 100, GetCommand);
    // initialize all estimations
    ResetStates();
    B_j << 1.0,0.0,
           0.0,1.0,
           0.0,0.0;
    Eigen::Matrix3f t_j_cross;
    Eigen::Vector2f r_j;
    Eigen::Vector2f mu_j;
    Eigen::Vector2f v_j;
    Eigen::Vector4f AttitudeQuaternionv;
<<<<<<< HEAD
    Eigen::Vector2f mu_j;
    Eigen::Vector2f temp;
=======
    Eigen::Vector3f temp;
>>>>>>> TCST_complete
    t_j_cross.setZero();
    DisplayParameters();// display parameters for checking...
    int check_flag;
    // check the data output on 
    ROS_INFO("Please check the parameter and setting, enter 1 to continue, else for quit: ");
    cin >> check_flag;
    if(check_flag != 1)
    {
        ROS_WARN("Found something wrong? terminating node ...");
        return -1;
    }

    ROS_INFO("Parameter OK, Start the interdrone communication and disturbance estimation...");
    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0.0;
    while(ros::ok())
    {
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time  - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;
        ros::spinOnce();
        switch (Command_Now.Mode) {
             case command_to_mavros_multidrone::Idle:
                // in idle mode: reset all state to zeros. 
                ResetStates();
                break;
             case command_to_mavros_multidrone::Payload_Stabilization: 
                if(_DroneState.mode != "OFFBOARD") {
                    ResetStates();// reset estimations if not in offboard mode
                } else {
                    /*Step 1, get payload information: */
                    // get the rotation matrix of the payload
                    for(int i = 0; i < 4 ;i++) {
                        AttitudeQuaternionv(i) = _DroneState.payload_quaternion[i];
                    }
                    R_IP = QuaterionToRotationMatrix(AttitudeQuaternionv); //
                    R_PI = R_IP.transpose(); // 
                    // get the linear and angular velocity of the payload
                    for (int i = 0; i < 3; i ++) {
                        v_p(i) = _DroneState.payload_vel[i];
                        omega_p(i) = _DroneState.payload_angular_vel[i];
                        omega_p_cross = Hatmap(omega_p);
                    }
                    /* Step 2 calculate force estimation */ 
                    // reset the accumulation of forces 
                    FT.setZero(); // total lift generated by props
                    FR.setZero(); // total moment generated by props
                    FR2.setZero();// auxiliary force in delta_R estimation
                    DT.setZero(); // total disturbance on quadrotors
                    DR.setZero(); // total disturbance moment on quadrotors
                    R1.setZero();
                    R2.setZero();
                    for (int i = 0; i < num_of_drones ; i ++) {
                        r_j = r_sq.col(i);
                        v_j = v_sq.col(i);
<<<<<<< HEAD
                        mu_j = kL * (r_j - rd_j.col(i));
=======
                        mu_j = kL * (r_j - rd_sq.col(i));
>>>>>>> TCST_complete
                        t_j_cross = Hatmap(t_sq.col(i));
                        DT += Delta_sq.col(i);
                        DR += t_j_cross * R_PI *  Delta_sq.col(i);
                        FT += f_L_sq.col(i);
                        // calculate B_j for each drone
                        float sq_r = r_j(0)*r_j(0) + r_j(1)*r_j(1);
                        if (cablelength_squared(i) - sq_r>0.01) {
                            B_j(2,0) =  - r_j(0)/sqrt((cablelength_squared(i) - sq_r));
                            B_j(2,1) =  - r_j(1)/sqrt((cablelength_squared(i) - sq_r));
                        } else {
                            B_j(2,0) = -0.1;
                            B_j(2,1) = -0.1;
                        }
                        temp = B_j * (v_j + mu_j);
                        R1 += a_j_sq(i) * temp;
<<<<<<< HEAD
                        R2 += a_j_sq(i) * E_j.segment(0,i*3,3,3).transpose() * R_PI * temp;
=======
                        R2 += a_j_sq(i) * E_j.block(0,i*3,3,3).transpose() * R_PI * temp;
>>>>>>> TCST_complete
                        BT += quadrotor_mass(i) * B_j * v_j;
                        FR += t_j_cross * (quadrotor_mass(i)*(omega_p_cross * R_PI *B_j*v_j - omega_p_cross * t_j_cross *omega_p -  R_PI * g_I) - R_PI * f_L_sq.col(i));
                        FR2 += quadrotor_mass(i)* t_j_cross * R_PI * B_j * v_j;
                    }
                    // calculate estimation based on the TCST paper. 
                    Delta_T_I +=  dt* ( Delta_T + FT + (M_q + payload_mass) * g_I + DT);
                    Delta_T = lambda_T * ( - Delta_T_I + (M_q + payload_mass) * v_p + R_IP * A.transpose() * omega_p +  BT);
                    Delta_R_I += dt *(A* omega_p_cross * R_PI * v_p + omega_p_cross * J_p * omega_p - Delta_R +  FR - DR);
                    Delta_R = lambda_R * (Delta_R_I  + A * R_PI * v_p + (J_p + J_q ) * omega_p +  FR2 );
                    // calculate effective disturbance: 
                    //Delta_pt = Delta_T - DT;
                    //Delta_pt = constrain_vector(Delta_pt, 10.0);
                    //Delta_rt = Delta_R - DR;
                    //Delta_rt = constrain_vector(Delta_rt, 5.0);
                    Delta_pt = constrain_vector(Delta_T, 10.0);
                    Delta_rt = constrain_vector(Delta_R, 5.0);
                }
                break;
            default:
                // in default mode, reset the estimation
                ResetStates();
                break;
         }
        PrintEstimation();
        // send the total disturbance estimation to each quadrotor
        _AddonForce.delta_Tx = Delta_pt(0);
        _AddonForce.delta_Ty = Delta_pt(1);
        _AddonForce.delta_Tz = Delta_pt(2);  
        _AddonForce.delta_Rx = Delta_rt(0);
        _AddonForce.delta_Ry = Delta_rt(1);  
        _AddonForce.delta_Rz = Delta_rt(2);
        // send cross-feeding forces to each quadrotor
        _AddonForce.R_1x = R1(0);
        _AddonForce.R_1y = R1(1);
        _AddonForce.R_1z = R1(2);
        _AddonForce.R_2x = R2(0);
        _AddonForce.R_2y = R2(1);
        _AddonForce.R_2z = R2(2);
        _AddonForce.header.stamp = ros::Time::now();
        pubAddonForce.publish(_AddonForce);
        rate.sleep();
    }
    return 0;
}
