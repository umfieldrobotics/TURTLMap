#include <ros/ros.h>
#include "onr_posegraph_backend_online/PosegraphBackendOnline.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posegraph_backend_online");
    // ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    // init with string file
    // pose_graph_backend::PosegraphBackendOnline posegraph_backend_online(nh, nh_private);
    std::string config_file;

    // get it as a ros parmeeter
    if (!ros::param::get("~config_file", config_file))
    {
        ROS_ERROR("Failed to get config file path");
        return -1;
    }

    // config_file = "./src/onr_posegraph_backend_online/param/br2config.yaml";
    pose_graph_backend::PosegraphBackendOnline posegraph_backend_online(config_file);

    // ros::spin();

    // return 0;
}