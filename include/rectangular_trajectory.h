/***************************************************************************************************************************
* rectangular_trajectory.h
*
* Author: Longhao Qian
*
* Update Time: 2020 7 30
*
* Introduction:  Rectangular trajectory
***************************************************************************************************************************/
#ifndef RECTANGULAR_TRAJECTORY_H  
#define RECTANGULAR_TRAJECTORY_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>

namespace trajectory{
    struct Rectangular_Trajectory_Parameter{
        float a_x;
        float a_y;
        float h;
        float v_x;
        float v_y;
        float center_x;
        float center_y;
        float center_z;
    };

    struct Reference_Path{
        Eigen::Vector3f P;
        Eigen::Vector3f n;
        float vd;
    };

    class Rectangular_Trajectory
    {
        public:
            Rectangular_Trajectory();
            ~ Rectangular_Trajectory();
            void LoadParameter(Rectangular_Trajectory_Parameter& parameter);
            void printf_param();
            void printf_result();
            Reference_Path UpdatePosition(Eigen::Vector3f& position);
        private:
            void DetermineState();
            int state; // indicate which path the quadrotor should follow
            // Parameter
            Rectangular_Trajectory_Parameter param_;
            Eigen::Vector3f position; // drone position
            float theta[4];// the angles for all turning points
            float theta_now;// the angle cooresponding to the current drone position
            int theta_output; // the angle for outputing reference path
            Reference_Path path[4];
            int target_theta[4];// the target angle cooresponding to each path.
    };

    Rectangular_Trajectory::Rectangular_Trajectory(){
        int state = 0;
    }

    Rectangular_Trajectory::~Rectangular_Trajectory(){

    }

    void Rectangular_Trajectory::LoadParameter(Rectangular_Trajectory_Parameter& parameter)
    {
        param_ = parameter;
        path[0].P(math_utils::Vector_X) = parameter.a_x/2.0 + parameter.center_x;
        path[0].P(math_utils::Vector_Y) = parameter.a_y/2.0 + parameter.center_y;
        path[0].P(math_utils::Vector_Z) = parameter.h + parameter.center_z;

        path[0].n(math_utils::Vector_X) = -1.0;
        path[0].n(math_utils::Vector_Y) = 0.0;
        path[0].n(math_utils::Vector_Z) = 0.0;
        path[0].vd = parameter.v_x;

        path[1].P(math_utils::Vector_X) = -parameter.a_x/2.0 + parameter.center_x ;
        path[1].P(math_utils::Vector_Y) = parameter.a_y/2.0 + parameter.center_y;
        path[1].P(math_utils::Vector_Z) = parameter.h + + parameter.center_z;

        path[1].n(math_utils::Vector_X) = 0.0;
        path[1].n(math_utils::Vector_Y) = -1.0;
        path[1].n(math_utils::Vector_Z) = 0.0;

        path[1].vd = parameter.v_y;

        path[2].P(math_utils::Vector_X) = -parameter.a_x/2.0 + parameter.center_x;
        path[2].P(math_utils::Vector_Y) = -parameter.a_y/2.0 + parameter.center_y;
        path[2].P(math_utils::Vector_Z) = parameter.h + parameter.center_z;

        path[2].n(math_utils::Vector_X) = 1.0;
        path[2].n(math_utils::Vector_Y) = 0.0;
        path[2].n(math_utils::Vector_Z) = 0.0;

        path[2].vd = parameter.v_x;

        path[3].P(math_utils::Vector_X) = parameter.a_x/2.0 + parameter.center_x;
        path[3].P(math_utils::Vector_Y) = -parameter.a_y/2.0 + parameter.center_y;
        path[3].P(math_utils::Vector_Z) = parameter.h + parameter.center_z;

        path[3].n(math_utils::Vector_X) = 0.0;
        path[3].n(math_utils::Vector_Y) = 1.0;
        path[3].n(math_utils::Vector_Z) = 0.0;

        path[3].vd = parameter.v_y;
        for(int i = 0;i<4;i++){
            theta[i] = atan2(path[i].P(math_utils::Vector_Y)-param_.center_y,path[i].P(math_utils::Vector_X)-param_.center_x);
        }
        target_theta[0] = 1;
        target_theta[1] = 2;
        target_theta[2] = 3;
        target_theta[3] = 0;
        theta_output = 0;
    }

    Reference_Path Rectangular_Trajectory::UpdatePosition(Eigen::Vector3f& position){
        theta_now = atan2(position(math_utils::Vector_Y)-param_.center_y,position(math_utils::Vector_X)-param_.center_x);// get the theta at the position
        DetermineState();// determine state
        // determine whether the drone is close to the turning point 
        Eigen::Vector3f n_q;
        n_q(math_utils::Vector_X) = cos(theta_now);
        n_q(math_utils::Vector_Y) = sin(theta_now);
        n_q(math_utils::Vector_Z) = 0.0;

        Eigen::Vector3f n_p;
        n_p(math_utils::Vector_X) = cos(theta[target_theta[state]]);
        n_p(math_utils::Vector_Y) = sin(theta[target_theta[state]]);
        n_p(math_utils::Vector_Z) = 0.0;

        if(n_q.cross(n_p)(math_utils::Vector_Z)<0.05)
        {
            // drone has reached the switching boundary
            theta_output = target_theta[state];
        }else{
            // not reaching the switching boundary yet
            theta_output = state;
        }
        // output reference path based on theta to follow
        return path[theta_output];
    }

    void Rectangular_Trajectory::DetermineState(){
        if(theta_now>0){
            if(theta_now>theta[1])
            {
                state = 1;
            }else if(theta_now>theta[0]){
                state = 0;
            }else{
                state = 3;
            }
        }else{
            if(theta_now<theta[2]){
                state = 1;
            }else if(theta_now<theta[3]){
                state = 2;
            }else{
                state = 3;
            }
        }
    }

    void Rectangular_Trajectory::printf_result()
    {
        cout <<">>>>>>>>>>>>>>> Rectrangular Trajectory State <<<<<<<<<<<<<<" <<endl;
        cout.setf(ios::fixed);

        cout.setf(ios::left);

        cout.setf(ios::showpoint);

        cout.setf(ios::showpos);

        cout<<setprecision(2);

        cout << " state: " << state <<"\n";
        cout << " theta_now: " << theta_now*57.3 << "(DEG)\n";
        cout << " theta_output: " << theta_output <<"\n";
    }

    void Rectangular_Trajectory::printf_param()
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Rectangular Trajectory Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout <<"reference path: " <<endl;
        for(int i = 0; i< 4; i++){
            cout << " Waypoint (m): " << path[i].P << endl;
            cout << " Direction : " <<path[i].n<<endl;
            cout << " Velocity(m/s): "<<path[i].vd<<endl;
            cout << " Theta(RAD): " << theta[i] <<endl;
        }
    }
}


#endif
