#include <ros/ros.h>

#include <iostream>
#include <LowPassFilter.h>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_tester");
    ros::NodeHandle nh;

    float T1 = 0.1;
    float T2 = 1;

    LowPassFilter LPF1;
    LowPassFilter LPF2;

    LPF1.set_Time_constant(T1);
    LPF2.set_Time_constant(T2);


    float dt = 1;

    float input1,input2;
    float output1,output2;

    while(ros::ok())
    {

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Input the input of the LPF1"<<endl;
        cin >> input1;

        cout << "Input the input of the LPF2"<<endl;
        cin >> input2;

        output1 = LPF1.apply(input1,dt);

        output2 = LPF2.apply(input2,dt);

        float _T1 = LPF1.get_Time_constant();
        float _T2 = LPF2.get_Time_constant();


        cout << "T for LPF1: "<< _T1 <<endl;
        cout << "T for LPF2: "<< _T2 <<endl;

        cout << "ouput for LPF1: "<< output1 <<endl;
        cout << "ouput for LPF2: "<< output2 <<endl;


        sleep(0.1);
    }

    return 0;
}
