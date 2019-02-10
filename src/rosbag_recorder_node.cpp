#include <ros/ros.h>
#include "bag_launcher.h"

using namespace bag_launcher_node;

int main(int argc, char** argv) {
    //initialize node
    ros::init(argc, argv, "rosbag_recorder_node");
    ros::NodeHandle nh("~");

    BLOptions options;

    //load param strings with no default value
    if(!nh.getParam("configuration_directory", options.configuration_directory)) {
        ROS_ERROR("Unable to start Bag Recorder Node. No configuration directory supplied.");
        return 0;
    }
    if(!nh.getParam("data_directory", options.data_directory)) {
        ROS_ERROR("Unable to start Bag Recorder Node. No data directory supplied.");
        return 0;
    }

    //load param strings with default values
    nh.param<std::string>("start_bag_topic",    options.record_start_topic, "/recorder/start");
    nh.param<std::string>("stop_bag_topic",     options.record_stop_topic,  "/recorder/stop");
    nh.param<std::string>("name_topic",         options.name_topic,         "/recorder/bag_name");
    nh.param<std::string>("heartbeat_topic",    options.heartbeat_topic,    "/recorder/heartbeat");

    //load bool params
    nh.param<bool>("publish_name",              options.publish_name,       true);
    nh.param<bool>("publish_heartbeat",         options.publish_heartbeat,  true);
    nh.param<bool>("default_record_all",        options.default_record_all, false);

    //load double param
    nh.param<double>("heartbeat_interval",      options.heartbeat_interval, 10);

    //assume we are running in unix and sanitize the directories
    if(options.configuration_directory.substr(options.configuration_directory.length()-1) != "/")
        options.configuration_directory += "/";
    if(options.data_directory.substr(options.data_directory.length()-1) != "/")
        options.data_directory += "/";

    //Print Configuration
    ROS_INFO("[Bag Recorder] Launching.");
    ROS_INFO("[Bag Recorder] Configurations located in %s.", options.configuration_directory.c_str());
    ROS_INFO("[Bag Recorder] Data directory located at %s.", options.data_directory.c_str());
    ROS_INFO("[Bag Recorder] Start Recording topic: %s.", options.record_start_topic.c_str());
    ROS_INFO("[Bag Recorder] Stop Recording topic: %s.", options.record_stop_topic.c_str());
    if(options.publish_name) {
        ROS_INFO("[Bag Recorder] Publishing bag names to %s.", options.name_topic.c_str());
    }
    if(options.publish_heartbeat) {
        ROS_INFO("[Bag Recorder] Publishing heartbeat every %.2f seconds to %s.", options.heartbeat_interval, options.heartbeat_topic.c_str());
    }

    //Make Bag Launcher node
    BagLauncher bag_launcher(nh, options);

    //main loop
    while(ros::ok()) {
        //bag_launcher's loop function
        bag_launcher.check_all();
        ros::spinOnce();
    }

}
