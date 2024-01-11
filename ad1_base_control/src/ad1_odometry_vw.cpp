#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int32MultiArray.h"
#include "ad1_msgs/PlantRobotInfo.h"

//AD1
const double PI = 3.14159265;
const double WHEELRADIUS = 0.0865; // Wheel radius in meters
const double DistancePerCount = (2 * PI * WHEELRADIUS) / 1500 ;// (2*pi*r)/ppr
const double WHEEL2WHEELWIDTH = 0.305; // Center of left tire to center of right tire base :0.33

ros::Subscriber sub_robotinfo;

// Create odometry data publishers
ros::Publisher odom_pub;

nav_msgs::Odometry odom;

bool init_last_position = false;
double last_position_x,last_position_y, last_angle ;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;

double position_x = 0.0;
double position_y = 0.0;

int initial_encoder_r = 0;
int initial_encoder_l = 0;

double last_distance_r = 0.0;
double last_distance_l = 0.0;

double g_distance_r = 0;
double g_distance_l = 0;

bool g_encoder_first = false;


tf2::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;


    float steering_angle_ = 0;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_ = 0;
    float x_pos_ = 0;
    float y_pos_ = 0;
    float heading_ = 0;

void robotinfo_cb(const ad1_msgs::PlantRobotInfo::ConstPtr& robotinfo)
{

    ros::Time current_time = ros::Time::now();
    static tf::TransformBroadcaster odom_broadcaster;

    linear_velocity_x_ = robotinfo->velocities[0];
    //linear_velocity_y_ = vel.linear_y;
    angular_velocity_z_ = robotinfo->velocities[1];

    vel_dt_ = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) ) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) ) * vel_dt_; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    odom_quat.setRPY(0,0,heading_);
/*
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = x_pos_;
    odom_trans.transform.translation.y = y_pos_;
    odom_trans.transform.translation.z = 0.0;
    //robot's heading in quaternion
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = current_time;
    //publish robot's tf using odom_trans object
    odom_broadcaster.sendTransform(odom_trans);
*/


    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    //angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_pub.publish(odom);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "ad1_odometry_vw");
  ros::NodeHandle nh;





  odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/wheel", 50);

  sub_robotinfo = nh.subscribe("plantrobot_info", 1000, robotinfo_cb);


  ros::Rate loop_rete(30);

  while(ros::ok())
  {

    ros::spinOnce();
    loop_rete.sleep();
  }

}
