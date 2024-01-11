#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//#include <stdio.h>
//#include <unistd.h>
//#include "kbhit.h"
#include <stdlib.h>
#include <ncurses.h>


// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
keyboard direction key
up key    : increase only linear speed by 0.1ms
down key  : decrease only linear speed by 0.1ms
left key  : increase only angular speed by 0.1r/s
right key : decrease only angular speed by 0.1r/s

space bar : stop

q to quit


)";

// Init variables
float speed(0.0); // Linear velocity (m/s)
float turn(0.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "ad1_teleop_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

  // Create Twist message
  geometry_msgs::Twist msg_cmd_vel;

  //char key(' ');
  int key_nr;

  //printf("%s", msg);

  //printf("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);

  //keyboard keyb;

  ros::Rate loop_rete(5);
  int key = 0;;
  WINDOW *win;

  // initialization
  win = initscr(); // new screen will be created
  nodelay(win, TRUE);
  noecho();
  printw("%s", msg); // instead of printf
  printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn); // instead of printf

  while(true){

      key = getch();
      /*
      if(key != -1)
      {
        //printw("%d", key);
        printw("KEY NAME : %s - %d\n", keyname(key),key);
      }*/

      if (key == '\033') { // if the first value is esc

          //keyb.getch();  // skip the [
         getch();
        switch(getch()) { // the real value
              case 'A':
                  // code for arrow up
                  speed += 0.1;
                  if(speed > 0.5)
                  {
                    speed = 0.5;
                  }

                   printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                  break;
              case 'B':
                  speed -= 0.1;
                  if(speed < -0.5)
                  {
                    speed = -0.5;
                  }
                   printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                  break;
              case 'C':
                  turn -= 0.1;
                  if(turn < -0.5)
                  {
                    turn = -0.5;
                  }

                   printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                  break;
              case 'D':


                  turn += 0.1;
                  if(turn > 0.5)
                  {
                   turn = 0.5;
                  }

                  printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
                  break;
          }
      }
      else if(key == 32)
      {
          speed = 0.0;
          turn = 0.0;
           printw("\rlinear velocity %f\tangular velocity %f | Awaiting command...\r", speed, turn);
      }
      else if( key == 'q')
      {
        printw("exit!!! \r\n");
        break;

      }



    msg_cmd_vel.linear.x=speed;
    msg_cmd_vel.angular.z=turn;
    pub_cmd_vel.publish(msg_cmd_vel);
    //refresh();

    ros::spinOnce();
    loop_rete.sleep();
  }
  delwin(win);
  endwin();
  refresh();
  return 0;
}
