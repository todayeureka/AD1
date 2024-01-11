#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"

//const float WHEEL2WHEELWIDTH = 0.31 ;// meter
//const float WHEELRADIUS = 0.08775 ; //meter

//AD1
const float WHEEL2WHEELWIDTH = 0.335 ;// meter
const float WHEELRADIUS = 0.1 ; //meter

ros::Publisher pub_motor_rpm;

std_msgs::Int16MultiArray motor_rpm;

void velocity_cb(const geometry_msgs::Twist::ConstPtr& cmd)
{

  //reference : https://en.wikipedia.org/wiki/Differential_wheeled_robot#Kinematics_of_Differential_Drive_Robots
  float rpm_r = (cmd->linear.x + (cmd->angular.z * WHEEL2WHEELWIDTH / 2))/WHEELRADIUS *9.54929; // 9.54929 : radian/s to rpm
  float rpm_l = (cmd->linear.x - (cmd->angular.z * WHEEL2WHEELWIDTH / 2))/WHEELRADIUS *9.54929;

  motor_rpm.data.clear();
  motor_rpm.data.push_back((short)rpm_l);
  motor_rpm.data.push_back((short)rpm_r);


  pub_motor_rpm.publish(motor_rpm);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ad1_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, velocity_cb);
  pub_motor_rpm = nh.advertise<std_msgs::Int16MultiArray>("motor_vel",1000); //for real robot

  ros::Rate loop_rete(10);

  ROS_INFO("ad1_control..");

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rete.sleep();
  }

  return 0;
}
