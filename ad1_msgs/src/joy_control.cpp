#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

#define BUZZER_PIN 18

//system()
#include <stdlib.h>
//fork()
#include <unistd.h>
#include <sys/wait.h>
//child process kill
#include <signal.h>

#include <iostream>
#include <string>
#include <vector>
//#include <experimental/filesystem>
#include <dirent.h>

//file
#include <iostream>
#include <fstream>

using std::cout; using std::endl;
using std::string; using std::vector;

#define PI 3.14159265

ros::Publisher g_pubVel;
int g_vel_v;
int g_vel_w;
int64_t g_left_right_value = 0;
int64_t g_up_down_value = 0;

float g_left_right_value2 = 0;
float g_up_down_value2 = 0;

int8_t g_left_right_flag = 0;   //1, 0, -1(left, center, right)
int8_t g_up_down_flag = 0;   //1, 0, -1(up, center, down)
int step = 1000;

/*Pan Tilt variable*/
ros::ServiceClient m_srvSetPt;

int g_nDirPan=0;
int g_nDirTilt=0;
int g_nPanTiltOffset = 10;

int nGoalPos=2048;//tilt8
int nGoalPos2=2048;//pan
int g_MinPositionLimit1 = 1023; //tilt
int g_MaxPositionLimit1 = 3073;
int g_MinPositionLimit2 = 1023; //pan
int g_MaxPositionLimit2 = 2200;

//int g_MinPositionLimit1 = 0; //tilt
//int g_MaxPositionLimit1 = 4095; 
//int g_MinPositionLimit2 = 0; //pan
//int g_MaxPositionLimit2 = 4095;

//joy stick
void DriveJoyCB(const sensor_msgs::Joy::ConstPtr& _joy)
{
    g_left_right_value2 = _joy->axes[0];
    g_up_down_value2 = _joy->axes[1];


    if(_joy->axes[0]>0.2f)
    {
        g_left_right_flag = 1;//left
    }
    else if(_joy->axes[0]<-0.2f)
    {
        g_left_right_flag = -1;//right
    }
    else {
        g_left_right_flag = 0;//center
    }

    if(_joy->axes[1]>0.2f)
    {
        g_up_down_flag = 1;//up
    }
    else if(_joy->axes[1]<-0.2f)
    {
        g_up_down_flag = -1;//down
    }
    else {
        g_up_down_flag = 0;//center
    }


    //tilt 상하
    if( _joy->axes[5]>0.2f)
        g_nDirTilt=-1;
    else if( _joy->axes[5]<-0.2f)
        g_nDirTilt=1;
    else {
        g_nDirTilt=0;
    }

    //pan 좌우
    if( _joy->axes[4]>0.2f)
        g_nDirPan=1;
    else if( _joy->axes[4]<-0.2f)
        g_nDirPan=-1;
    else {
        g_nDirPan=0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_control");
    ros::NodeHandle nh;

    ros::Subscriber subJoy=nh.subscribe("/joy",1000, DriveJoyCB);
//    ros::Publisher g_pubVel = nh.advertise<std_msgs::Int64MultiArray>("/cmd_vel",1000);
    ros::Publisher g_pubVel = nh.advertise<std_msgs::Float32MultiArray>("/cmd_motor",1000);
    ros::Publisher g_pubPos = nh.advertise<std_msgs::Float32MultiArray>("/cmd_dynamixel",1000);

    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        /*drive*/
        std_msgs::Float32MultiArray vel;

        if(g_left_right_flag == 1)
        {
            g_left_right_value += step;
        }
        else if(g_left_right_flag == -1)
        {
            g_left_right_value -= step;
        }

        if(g_up_down_flag == 1)
        {
            g_up_down_value += step;
        }
        else if(g_up_down_flag == -1)
        {
            g_up_down_value -= step;
        }

        vel.data.push_back(g_left_right_value2*2);
        vel.data.push_back(g_up_down_value2*(-2));

        g_pubVel.publish(vel);

        /*PT camera*/
        std_msgs::Float32MultiArray pos;

        if(nGoalPos>=g_MinPositionLimit1 && nGoalPos<=g_MaxPositionLimit1)
        {
             if(g_nDirPan==-1)
                nGoalPos+=g_nPanTiltOffset;
             else  if(g_nDirPan==1)
                 nGoalPos-=g_nPanTiltOffset;
        }
        if(nGoalPos>g_MaxPositionLimit1)
            nGoalPos=g_MaxPositionLimit1;
        if(nGoalPos<g_MinPositionLimit1)
            nGoalPos=g_MinPositionLimit1;

        if(nGoalPos2>=g_MinPositionLimit2 && nGoalPos2<=g_MaxPositionLimit2)
        {
             if(g_nDirTilt==-1)
                nGoalPos2+=g_nPanTiltOffset;
             else  if(g_nDirTilt==1)
                 nGoalPos2-=g_nPanTiltOffset;
        }
        if(nGoalPos2>g_MaxPositionLimit2)
            nGoalPos2=g_MaxPositionLimit2;
        if(nGoalPos2<g_MinPositionLimit2)
            nGoalPos2=g_MinPositionLimit2;

        pos.data.push_back(nGoalPos);
        pos.data.push_back(nGoalPos2);

        g_pubPos.publish(pos);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}
