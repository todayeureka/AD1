#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ad1_tf");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::Quaternion q;
  tf::Quaternion q1;



 q.setRPY(0,0,1.570796); //90*(3.141592/180.0)
 q1.setRPY(0,0,0);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
  
        broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(
            q1,
            tf::Vector3(0.0, 0.0, 0.05)),
            ros::Time::now(),
            "base_footprint",
            "base_link"));

      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(
            q,
            tf::Vector3(0.20, 0.0, 0.05)),
            ros::Time::now(),
            "base_link",
            "base_scan"));


    r.sleep();
  }
}
