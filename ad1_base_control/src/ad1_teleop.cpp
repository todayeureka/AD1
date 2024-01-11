#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"



ros::Publisher pub_cmd_vel;
ros::Subscriber sub_joy;

float g_fVel=0;
float g_fw=0;

geometry_msgs::Twist msg_cmd_vel;

void joystic_cb(const sensor_msgs::Joy::ConstPtr& joy)
{
  //float fScale=((1+joy->axes[2])/2.0) * 3.3;
  g_fVel = joy->axes[1] /2; // max speed : 0.5w m/s
  g_fw = joy->axes[0]   ;   // max speed : 1 r/s
  //g_fw = joy->axes[7]/2;   // max speed : 0.5 r/s

  msg_cmd_vel.linear.x=g_fVel;
  msg_cmd_vel.angular.z=g_fw;
  pub_cmd_vel.publish(msg_cmd_vel);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ad1_teleop");
  ros::NodeHandle nh;



  sub_joy = nh.subscribe("joy", 1000, joystic_cb);
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("joy/cmd_vel",1000); //for real robot

  ros::Rate loop_rete(10);

  while(ros::ok())
  {

    //msg_cmd_vel.linear.x=g_fVel;
    //msg_cmd_vel.angular.z=g_fw;
    //pub_cmd_vel.publish(msg_cmd_vel);
    ros::spinOnce();
    loop_rete.sleep();
  }

  return 0;
}
