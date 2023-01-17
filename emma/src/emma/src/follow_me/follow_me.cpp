/** Simple example of robot following human.
 * Robot stands still and detects movement, 
 * it moves into the direction of the movement.
 * 
 * First, run
 *   roslaunch turtlebot_bringup minimal.launch
 *   roslaunch rplidar_ros rplidar.launch
 * and then
 *   rosrun emma follow_me
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "/home/mustar/kobuki/devel/include/kobuki_msgs/Sound.h"

#include <sstream>

const int LASER_WINDOW_WIDTH = 90;
const double DISTANCE_TO_STOP = 0.3;   // in meters
const double MIN_DETECT_ANGLE = 7;     // in degrees
const double OUTLIER_THRESHOLD = 0.5;  // in meters
const double MIN_ACCEPTABLE_Y_DISTANCE = 0.7; // in meters
const double MAX_ACCEPTABLE_Y_DISTANCE = 1.7; // in meters


static double current_linear_vel = 0;
static double current_angular_vel = 0;

ros::Publisher *speed_pub;
ros::Publisher *sound_pub;

void stop_robot()
{
	geometry_msgs::Twist msg;	
	msg.linear.x = 0;
	msg.angular.z = 0;
	speed_pub->publish(msg);
}  

void move_robot(double speed, double angular_speed)
{
	geometry_msgs::Twist msg;	
	msg.linear.x = speed;
	msg.angular.z = angular_speed;
	speed_pub->publish(msg);
}  

void button_beep()
{
	kobuki_msgs::Sound beep;
	beep.value = 4;
	sound_pub->publish(beep);
}  

int is_outlier(double *a, int i)
{
	if ((i == 0) || (i == 359)) return 0;
	if ((fabs(a[i] - a[i - 1]) > OUTLIER_THRESHOLD) &&
	    (fabs(a[i] - a[i + 1]) > OUTLIER_THRESHOLD)) return 1;
	return 0;
}
    
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	static int count = 0;
    static double last_scan[LASER_WINDOW_WIDTH];
    static double current_scan[LASER_WINDOW_WIDTH];
    static int in_movement = 0;
    static int next_scan_is_first = 1;
    
    if (in_movement)
    {
		in_movement--;
		move_robot(current_linear_vel, current_angular_vel);
	    if (in_movement == 0) { stop_robot(); count = -2; }
		return;
    }
        
    if (count++ == 4)  // seems we have 8 scans per second, look every half a second
    {
		count = 0;
		int j = 0;
		for (int i = 360 - LASER_WINDOW_WIDTH / 2; i < 360; i++)
		   current_scan[j++] = msg->ranges[i];
		
		for (int i = 0; i < LASER_WINDOW_WIDTH / 2; i++)
		   current_scan[j++] = msg->ranges[i];

		int in_sequence = 0;
		double partial_sum_x = 0;
		double partial_sum_y = 0;
		int partial_count = 0;
		double sum_x = 0;
		double sum_y = 0;
		int sum_count = 0;
		   
		if (!next_scan_is_first)
		{
			for (int i = 0; i < LASER_WINDOW_WIDTH; i++)
			{
				double change = fabs(current_scan[i] - last_scan[i]);
				
				if (!std::isfinite(current_scan[i])) continue;
				if (is_outlier(current_scan, i)) continue;
				
				if ((change > 0.2) && (i < LASER_WINDOW_WIDTH - 1))
				{
					if (!in_sequence) in_sequence = 1;
					partial_sum_y += current_scan[i];
					partial_sum_x += i;
					partial_count ++;
				}
				else
				{
					if (in_sequence && (partial_count >= MIN_DETECT_ANGLE))
					{
						sum_x += partial_sum_x;
						sum_y += partial_sum_y;
						sum_count += partial_count;
						partial_sum_x = 0;
						partial_sum_y = 0;
						partial_count = 0;
						in_sequence = 0;
					}
				}
			}  
		}
		else next_scan_is_first = 0;
		ROS_INFO("movement size=%d", sum_count);
		
		memcpy(last_scan, current_scan, sizeof(double) * LASER_WINDOW_WIDTH);
		if (sum_count < MIN_DETECT_ANGLE) return;
		
		sum_x /= sum_count;
		sum_y /= sum_count;
		
		ROS_INFO("detected movement: x=%.2lf y=%.2lf", sum_x, sum_y);
		
		if ((sum_y > MIN_ACCEPTABLE_Y_DISTANCE) && (sum_y < MAX_ACCEPTABLE_Y_DISTANCE)) return;
		
		if (sum_y <= MIN_ACCEPTABLE_Y_DISTANCE) 
		{
			//target_heading = (sum_x - 45) / 180.0 * M_PI;
			current_linear_vel = -0.2;
			current_angular_vel = 0; //-target_heading / 3;
			move_robot(current_linear_vel, current_angular_vel);   		
			in_movement = 10;	
			next_scan_is_first = 1;
		}
		else if ((sum_y >= MAX_ACCEPTABLE_Y_DISTANCE)) 
		{
			double target_heading = (sum_x - 45) / 180.0 * M_PI;
			current_linear_vel = 0.25;
			current_angular_vel = -target_heading / 2;
			move_robot(current_linear_vel, current_angular_vel);   		
			in_movement = 10; 
			next_scan_is_first = 1;
		}
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_me");

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

