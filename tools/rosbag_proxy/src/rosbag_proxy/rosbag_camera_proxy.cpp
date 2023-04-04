#include "web_video_server/web_video_server.h"

#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_camera_proxy");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    web_video_server::WebVideoServer server(nh, private_nh);
    server.spin();
    return 0;
}