#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosutil/PublisherHandle.hpp"

#include <sstream>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "publisher");

    ros::NodeHandle nh;

    rosutil::PublisherHandle publisherHandle(nh, "chatter", 100);

    ros::Rate loop_rate(1):

    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;

        std::ostringstream oss;
        oss << "hello world " << count;
        msg.data = oss.str();

        ROS_INFO("publishing %s", msg.data.c_str());

        publisherHandle.publish(msg);

        ros::spinOnce();

        loop_date.sleep();
        ++count;
    }

    return 0;
}
