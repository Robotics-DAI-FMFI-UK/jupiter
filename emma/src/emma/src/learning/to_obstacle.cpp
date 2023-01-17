/** Simple example of robot movement and lidar obstacle detection.
 * Robot starts moving, and stops 30cm in front of an obstacle.
 * It uses the 10 front lidar rays for obstacle detection.
 * Infinity values are ignored, otherwise they are averaged.
 * 
 * First, run
 *   roslaunch turtlebot_bringup minimal.launch
 *   roslaunch rplidar_ros rplidar.launch
 * and then
 *   rosrun emma to_obstacle
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "/home/mustar/kobuki/devel/include/kobuki_msgs/Sound.h"

#include <sstream>

const int LASER_WINDOW_WIDTH = 5;
const double DISTANCE_TO_STOP = 0.3;   // in meters

ros::Publisher *speed_pub;
ros::Publisher *sound_pub;

void stop_robot()
{
	geometry_msgs::Twist msg;	
	msg.linear.x = 0;
	msg.angular.z = 0;
	speed_pub->publish(msg);
}  

void start_robot()
{
	geometry_msgs::Twist msg;	
	msg.linear.x = 0.1;
	msg.angular.z = 0;
	speed_pub->publish(msg);
}  

void button_beep()
{
	kobuki_msgs::Sound beep;
	beep.value = 4;
	sound_pub->publish(beep);
}  

void add_to_avg(double &sum, int &count, double x)
{
	if (std::isfinite(x))
    {

		sum += x;
		count++;
        ROS_INFO("added: %.2lf to %.2lf, cnt=%d", x, sum, count);
	}
}
    
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  static int stopped = 0;
	
  if (!stopped) start_robot();
	
  int count = 0;
  double sum = 0.0;
  for (int i = 0; i < LASER_WINDOW_WIDTH; i++)
    add_to_avg(sum, count, msg->ranges[i]);
  for (int i = 360 - LASER_WINDOW_WIDTH; i < 360; i++)
    add_to_avg(sum, count, msg->ranges[i]);
  if (count == 0) return;
  sum /= count;
  ROS_INFO("forward dst: %.2lf, count=%d", sum, count);
  if ((sum < DISTANCE_TO_STOP) && (!stopped))
  {
	stopped = 1;
    stop_robot();
    button_beep();
    ROS_INFO("stopping robot due to obstacle detection %.2lf", sum);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "to_obstacle");

  ros::NodeHandle n;
  
  ros::Publisher speed_pub_local = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  ros::Publisher sound_pub_local = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 10);
  speed_pub = &speed_pub_local;
  sound_pub = &sound_pub_local;

  ros::Subscriber sub = n.subscribe("/scan", 100, scanCallback);

  ros::Rate loop_rate(10);
  
  ros::spin();
  
  return 0;
}

