#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <tuple>
#include <cstdio>
#include "jupiter_arm_kinematics.h"

using namespace std;

/**
 * example of inverse kinematics for jupiter arm 
 * Program asks for [x,y,z] of target point, x being
 * perpendicular to robot heading increasing right,
 * y is in direction of robot heading, z is vertical,
 * starting in the axle of servo of second arm joint,
 * l1=0.105, l2=0.105, l3=0.1 are lenths of the three
 * arm segments. The last parameter is the angle displacement
 * of the last segment (angle of approaching the object,
 * 0deg is vertical downwards, 90deg is in heading direction)
 * First, run
 *   roslaunch rchomeedu_arm arm.launch
 * and then
 *   rosrun emma arm_test
 * 
 */


void rossleep(int n)
{
  for (int i = 0; i < n * 10; i++)
  {
      ros::spinOnce();
      usleep(100000);
  }
}
   	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");

  ros::NodeHandle n;
  ros::Publisher arm0_pub = n.advertise<std_msgs::Float64>("/waist_controller/command", 100);
  ros::Publisher arm1_pub = n.advertise<std_msgs::Float64>("/shoulder_controller/command", 100);
  ros::Publisher arm2_pub = n.advertise<std_msgs::Float64>("/elbow_controller/command", 100);
  ros::Publisher arm3_pub = n.advertise<std_msgs::Float64>("/wrist_controller/command", 100);
  ros::Publisher arm4_pub = n.advertise<std_msgs::Float64>("/hand_controller/command", 100);

  ros::Rate loop_rate(10);
  
  int q = 1;
  int cnt = 0;

    double x,y,z,l1,l2,l3,pi;

    printf("x=");
    scanf("%lf", &x);
    printf("y=");
    scanf("%lf", &y);
    printf("z=");
    scanf("%lf", &z);
    printf("l1=");
    scanf("%lf", &l1);
    printf("l2=");
    scanf("%lf", &l2);
    printf("l3=");
    scanf("%lf", &l3);
    printf("pi[deg]=");
    scanf("%lf", &pi);
    pi *= M_PI / 180.0;

    tuple<double, double, double, double> angles = findArmAngles(x,y,z,l1,l2,l3,pi);

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
    rossleep(3);
          
    msg.data = get<1>(angles);
    arm1_pub.publish(msg);
    rossleep(3);
    
    msg.data = get<2>(angles);
    arm2_pub.publish(msg);
    rossleep(3);
    
    msg.data = get<3>(angles);
    arm3_pub.publish(msg);


  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     if (cnt++ == 30)
     {
		 cnt = 0;
     }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

