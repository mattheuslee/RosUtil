#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosutil/PublisherHandle.hpp"

#include <sstream>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "publisher");

    ros::NodeHandle nh;

    rosutil::PublisherHandle<std_msgs::String> publisherHandle(nh, "chatter", 100);

    ros::Rate loopRate(1);

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::ostringstream oss;
        oss << "hello world " << count;
        msg.data = oss.str();

        ROS_INFO("publishing %s", msg.data.c_str());

        publisherHandle.publish(msg);

        ros::spinOnce();

        loopRate.sleep();
        ++count;
    }

    return 0;
}
