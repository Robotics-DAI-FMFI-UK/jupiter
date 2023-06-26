#include <sys/time.h>
#include <time.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <tuple>
#include <cstdio>
#include "jupiter_arm_kinematics.h"

using namespace std;

unsigned long usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000000UL * tv.tv_sec + tv.tv_usec;
}

void rossleep(int n)
{
  unsigned long tm = usec();

  while (usec() - tm < n * 1000000UL)
  {
      ros::spinOnce();
  }
}
   	
  ros::Publisher arm0_pub;
  ros::Publisher arm1_pub;
  ros::Publisher arm2_pub;
  ros::Publisher arm3_pub;
  ros::Publisher arm4_pub;

void init_move_arm_to_xyz(ros::NodeHandle n)
{
  arm0_pub = n.advertise<std_msgs::Float64>("/waist_controller/command", 100);
  arm1_pub = n.advertise<std_msgs::Float64>("/shoulder_controller/command", 100);
  arm2_pub = n.advertise<std_msgs::Float64>("/elbow_controller/command", 100);
  arm3_pub = n.advertise<std_msgs::Float64>("/wrist_controller/command", 100);
  arm4_pub = n.advertise<std_msgs::Float64>("/hand_controller/command", 100);
}

void move_arm_to_xyz(double x, double y, double z, double gamma)
{
   double l1 = 0.105;
   double l2 = 0.105;
   double l3 = 0.120;

   tuple<double, double, double, double> angles = findArmAngles(x,y,z,l1,l2,l3,gamma);

   if (isnan(get<0>(angles)) ||
       isnan(get<1>(angles)) ||
       isnan(get<2>(angles)) ||
       isnan(get<3>(angles))) 
   {
     printf("Nan in angles, not moving arm - is target too far?\n");
     return;
   }

    printf("---------------\na1=%.3lf (%.1lf deg)\na2=%.3lf (%.1lf deg)\na3=%.3lf (%.1lf deg)\na4=%.3lf (%.1lf deg)\n",
              get<0>(angles),
              get<0>(angles) * 180.0 / M_PI,
              get<1>(angles),
              get<1>(angles) * 180.0 / M_PI,
              get<2>(angles),
              get<2>(angles) * 180.0 / M_PI,
              get<3>(angles),
              get<3>(angles) * 180.0 / M_PI);

    std_msgs::Float64 msg;

    msg.data = -get<0>(angles);
    arm0_pub.publish(msg);
    //rossleep(1);
          
    msg.data = get<1>(angles);
    arm1_pub.publish(msg);
    //rossleep(1);
    
    msg.data = get<2>(angles);
    arm2_pub.publish(msg);
    //rossleep(1);
    
    msg.data = get<3>(angles);
    arm3_pub.publish(msg);

}

void close_hand()
{
    std_msgs::Float64 msg;
    msg.data = 0.35;
    arm4_pub.publish(msg);
}

void open_hand()
{
    std_msgs::Float64 msg;
    msg.data = -0.2;
    arm4_pub.publish(msg);
}
