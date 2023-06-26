/** Simple example of saving a pair of depth map to a text file and rgb image to a png file taken from 3D camera
 *
 * First, run
 *   roslaunch turtlebot_bringup minimal.launch
 *   roslaunch emma double_astra.launch
 * and then 
 *   rosrun emma save_depth CAMERA_NUMBER
 *
 * CAMERA_NUMBER is 1 or 2
 * image is 640x480, but left border 40px and right border 49px should be ignored (black or something else...)
 *                   and top border is 26px, 34px
 *  column 591 contains some status data (???)
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace std;

static int save_rgb = 0;
static char rgb_file_name[100];

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Prevod z ROS formátu na OpenCV formát
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (save_rgb)
        {
          imwrite(rgb_file_name, img);          
          printf("RGB image saved to %s\n", rgb_file_name);
          exit(0);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static int depth_counter = 0;
    depth_counter++;
    if (depth_counter == 50)
    {
       char depth_file_name[100];
       time_t t;
       time(&t);
       sprintf(depth_file_name, "depth_map_%ld.txt", t);
       sprintf(rgb_file_name, "rgb_%ld.png", t);
       FILE *f = fopen(depth_file_name, "w+");
       int index = 0;
       for (unsigned int i = 0; i < msg->height; i++)
       {
         for (unsigned int j = 0; j < msg->width; j++)
         {
            fprintf(f, "%hu", msg->data[index] + (((uint16_t)msg->data[index + 1]) << 8));
            index += 2;
            if (j < msg->width - 1) fprintf(f, " ");
         }
         fprintf(f, "\n");
       }
       fclose(f);
       printf("Depth image saved to %s\n", depth_file_name);
       save_rgb = 1;
    }
}

void usage()
{
       printf("usage: rosrun emma save_depth CAMERA_NUMBER\n");
       exit(1);
}

int main(int argc, char** argv)
{
    const char *use_camera_rgb, *use_camera_depth;

    if (argc != 2)
    {
       usage();
    }
    if (strcmp(argv[1], "1") == 0)
    {
       use_camera_rgb = "/camera/rgb/image_raw";
       use_camera_depth = "/camera/depth/image_raw";
    }
    else if (strcmp(argv[1], "2") == 0)
    {
       use_camera_rgb = "/camera_top/rgb/image_raw";
       use_camera_depth = "/camera_top/depth/image_raw";
    }
    else usage();

    ros::init(argc, argv, "depth_map_node");
    ros::NodeHandle nh;

    // Prihlásenie na odber obrazového topicu
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(use_camera_rgb, 1, rgbImageCallback);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Image>(use_camera_depth, 1, depthImageCallback);

    // Spustenie cyklu spracovania udalostí
    ros::spin();

    return 0;
}

