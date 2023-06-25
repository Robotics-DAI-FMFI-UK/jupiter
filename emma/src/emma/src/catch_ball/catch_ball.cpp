/** Grabs ball seen in bottom camera view by arm, if it is reachable
 *
 * First, run
 *   roslaunch turtlebot_bringup minimal.launch
 *   roslaunch emma double_astra.launch
 *   roslaunch rchomeedu_arm arm.launch
 * and then
 *   rosrun emma catch_ball
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "find_brick.h"
#include "arm_to_xyz.h"
#include "camera_to_xyz.h"
#include "camera_to_arm.h"

using namespace std;

static int want_to_move_arm = 0;

static	int sirka_kocky;
static	int vyska_kocky;
static	int velkost_kocky;
static	int riadok;
static	int stlpec;
	

void find_cube(cv::Mat img)
{
	if (!img.isContinuous())
	{
		cout << "image is not a continuous matrix!!!\n";
		return;
	}
	
	buffer = img.ptr();
	
    najdi_kocku(&sirka_kocky, &vyska_kocky, &velkost_kocky, &riadok, &stlpec);
    
    if ((riadok < 0) || (riadok >= vyska) || (stlpec < 0) || (stlpec >= sirka)) return;
    
    printf("kocka: [%d,%d], velkost=%d, rozmery = [%d,%d]\n", stlpec, riadok, velkost_kocky, sirka_kocky, vyska_kocky);
	
}


int ball_reachable(double x, double y, double z)
{
    return (y < 0.5) && (fabs(x) < 0.15) && (fabs(z) < 0.25);
}

void move_arm_to_ball()
{
    printf("moving arm to ball...\n");
    move_arm_to_xyz(0, 0.10, 0.15, M_PI / 2);
    rossleep(3);
    double ball_x, ball_y, ball_z;
    calculate_position(stlpec, riadok, sirka_kocky, vyska_kocky, &ball_x, &ball_y, &ball_z);
    double arm_x, arm_y, arm_z;
    transform_camera_to_arm_coordinates(ball_x, ball_y, ball_z, &arm_x, &arm_y, &arm_z);
    if (ball_reachable(arm_x, arm_y, arm_z))
    {
         move_arm_to_xyz(arm_x, arm_y, arm_z, 30 / 180.0 * M_PI);
         rossleep(2);
         close_hand();
         rossleep(1);
         move_arm_to_xyz(0.22, 0.0, 0.15, M_PI / 2);
         rossleep(2);
         open_hand();
         rossleep(1);
         printf("done.\n");
    }
    else
    {
       printf("ball is not currently reachable\n");
    }
    want_to_move_arm = 0;  
}

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    { 
		//cout << "RGB: ";
		//cout << "w:" << msg->width << endl;
        //cout << "h:" << msg->height << endl;
        sirka = msg->width;
        vyska = msg->height;
        
        // Prevod z ROS formátu na OpenCV formát
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        //printf("call find_cube()\n");
        find_cube(img);
        cv::imshow("rgb", img);
        int k = cv::waitKey(1);
        if (k == ' ')
        {
           move_arm_to_ball();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_map_node");
    ros::NodeHandle nh;

    // Prihlásenie na odber obrazového topicu
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, rgbImageCallback);

    init_move_arm_to_xyz(nh);
    init_calculate_position();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (want_to_move_arm)
        move_arm_to_ball();
    }


    return 0;
}

