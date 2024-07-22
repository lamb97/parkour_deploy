#include <ros/ros.h>
#include "ImageNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_node");
    ros::NodeHandle nh;
    std::string depth_image_topic;
    nh.getParam("/camera/depth_image_topic", depth_image_topic);
    ROS_INFO("Parameter depth_image_topic default: %s", depth_image_topic.c_str());
    std::string obs_info_topic = "/obs_info";
    ImageNode node(nh, depth_image_topic, obs_info_topic);

    if (!node.init(nh))
    {
        ROS_ERROR("Failed to initialize ImageNode");
        return -1;
    }

    ros::AsyncSpinner spinner(2); // 使用两个线程
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
