#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

/**
 simple example of controlling arm by publishing messages to dynamixel
 * arm controller. First, run
 *   roslaunch rchomeedu_arm arm.launch
 * and then
 *   rosrun emma move_arm
 * 
 * the angle is in radians, we send alternating
 * requests to move waist to positions 60deg, 120deg 
 * every 3 seconds.
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");

  ros::NodeHandle n;
  ros::Publisher arm_pub = n.advertise<std_msgs::Float64>("/waist_controller/command", 100);

  ros::Rate loop_rate(10);
  
  int q = 1;
  int cnt = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     if (cnt++ == 30)
     {
		 cnt = 0;
		std_msgs::Float64 msg;

		msg.data = M_PI / 2.0 + q * M_PI / 6;
		q *= -1;

		ROS_INFO("%.3f", msg.data);

		arm_pub.publish(msg);
	}
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

