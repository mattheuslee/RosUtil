#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosutil/rosutil.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "subscriber");

    ros::NodeHandle nh;

    rosutil::SubscriberHandle<std_msgs::String> subscriberHandle(nh, "chatter", 100);

    while (ros::ok()) {
        ros::spinOnce();

        if (subscriberHandle.is_updated()) {
            ROS_INFO("heard %s", subscriberHandle.last_message().data.c_str());
        }

    }

    return 0;
}

