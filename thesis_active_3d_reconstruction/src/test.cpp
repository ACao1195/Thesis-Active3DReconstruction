#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
}