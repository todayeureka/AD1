#ifndef FMU_SOCKET_H
#define FMU_SOCKET_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/sem.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sys/timeb.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <signal.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "tf2/LinearMath/Matrix3x3.h"

#include "server_utils.h"
#include "batcam_base_pkg/FmuID.h"
#include "batcam_base_pkg/CmdFmu.h"

#define PI 3.14159265
#define EARTH_R 6371000 //meter

using namespace std;

#endif // FMU_SOCKET_H
