#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"



ros::Publisher pub_cmd_vel;
ros::Subscriber sub_web_control;

float g_fVel=0;
float g_fw=0;

geometry_msgs::Twist msg_cmd_vel;

void web_cb(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

  msg_cmd_vel.linear.x=cmd_vel->linear.x / 2;
  msg_cmd_vel.angular.z= cmd_vel->angular.z / 2;
  pub_cmd_vel.publish(msg_cmd_vel);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ad1_teleop_web");
  ros::NodeHandle nh;



  sub_web_control = nh.subscribe("web/cmd_vel", 1000, web_cb);
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("web/cmd_vel_low",1000); //for real robot

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
