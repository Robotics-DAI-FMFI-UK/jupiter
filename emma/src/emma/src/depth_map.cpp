/** Simple example of showing depth map from 3D camera
 *
 * First, run
 *   roslaunch turtlebot_bringup minimal.launch
 *   roslaunch turtlebot_bringup 3dsensor.launch
 * and then
 *   rosrun emma depth_map
 *
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

void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Prevod z ROS formátu na OpenCV formát
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::imshow("rgb", img);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cout << msg->header << endl;
        cout << "w:" << msg->width << endl;
        cout << "h:" << msg->height << endl;
        cout << "enc:" << msg->encoding << endl;
        cout << "isbe:" << msg->is_bigendian << endl;
        cout << "step:" << msg->step << endl;
        cout << "data:" << msg->data.size() << endl;
        uint16_t max = 0;
        uint64_t sum = 0;
        int col = -1;
        for (unsigned int i = 0; i < msg->data.size(); i+=2)
        {
          col++;
          if (col == 640) col = 0;

          uint16_t a = msg->data[i];
          uint16_t b = msg->data[i + 1];
          uint16_t value = ((b << 8) | a);

          int row = i / 1280;
          if (col < 40) continue;
          if (col >= 591) continue;
          if (row < 26) continue;
          if (row >= 446) continue;

          if (value > max) max = value;
          sum += value;
        }
        cout << "max:" << max << endl;
        cout << "avg:" << (sum / msg->data.size() * 2) << endl;

        double q = 65535 / max;

        sensor_msgs::Image *msg2 =  new sensor_msgs::Image;
        *msg2 = *msg;

        //vector<uint8_t> newdata(msg->data.size());
        for (unsigned int i = 0; i < msg->data.size(); i+=2)
        {
          uint16_t a = msg->data[i];
          uint16_t b = msg->data[i + 1];
          uint32_t value = ((b << 8) | a);
          value *= 0.5 + q;
          if (value > 65535) value = 65535;
          a = value & 255;
          b = value >> 8;
          msg2->data[i] = a;
          msg2->data[i + 1] = b;
        }

        sensor_msgs::ImageConstPtr msg2ptr(msg2);

        // Prevod z ROS formátu na OpenCV formát
        cv::Mat img = cv_bridge::toCvShare(msg2ptr, "16UC1")->image;

        cv::imshow("depth", img);
        cv::waitKey(1);
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
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, depthImageCallback);

    // Spustenie cyklu spracovania udalostí
    ros::spin();

    return 0;
}

