/// @file bag_launcher.cpp
/// @author joshs333@live.com
/// @details this is used as an overhead to the BagRecorder class. This operates
/// as a ROS node and loads different configs for the BagRecorder and launches
/// the bag recorder with this config and stops it by published command.
#include "bag_launcher.h"
#include <fstream>

namespace bag_launcher_node {

/**
* @brief BLOptions() initializes a BagLauncher options struct with default vals
*/
BLOptions::BLOptions():
    configuration_directory("/data/"),
    data_directory("/media/data"),
    record_start_topic("/bag/start"),
    record_stop_topic("/bag/stop"),
    publish_name(true),
    name_topic("/bag/name"),
    publish_heartbeat(true),
    heartbeat_topic("/bag/heartbeat"),
    heartbeat_interval(10)
{}

/**
* @brief BagLauncher() constructor makes BagLauncher node object
* @param [in] nh ros nodehandle to use
* @param [in] options options to generate the node from
* @details Initializes subscribers and variables to run node.
* Initializes name publisher if publish_name is true.
*/
BagLauncher::BagLauncher(ros::NodeHandle nh, BLOptions options):
    nh_(nh), config_location_(options.configuration_directory), data_folder_(options.data_directory),
    record_start_subscriber_(nh.subscribe(options.record_start_topic, 10, &BagLauncher::Start_Recording, this)),
    record_stop_subscriber_(nh.subscribe(options.record_stop_topic, 10, &BagLauncher::Stop_Recording, this)),
    publish_name_(options.publish_name), publish_heartbeat_(options.publish_heartbeat),
    heartbeat_topic_(options.heartbeat_topic), heartbeat_interval_(options.heartbeat_interval) {

    if(options.publish_name) {
        name_publisher_ = nh.advertise<bag_recorder::Rosbag>(sanitize_topic(options.name_topic), 5);
    }
}

/**
* @brief ~BagLauncher() destructs the bag launcher object safely
* @details stops each recorder immediately
*/
BagLauncher::~BagLauncher(){
    for(std::map<std::string, std::shared_ptr<BagRecorder>>::iterator it = recorders_.begin(); it != recorders_.end(); ++it)
        it->second->immediate_stop_recording();
}

/**
* @brief Start_Recording() starts a bag of a certain config type and a given name
* @param [in] msg ROS msg containing config type and bag name
* @details creates recorder of given type if it does not exist in map already.
* Starts the recorder recording. Generates heartbeat publisher if enabled,
* publishes bag name if enabled.
*/
void BagLauncher::Start_Recording(const bag_recorder::Rosbag::ConstPtr& msg){
    //find the recorder under that name if it exists already
    std::map<std::string, std::shared_ptr<BagRecorder>>::iterator recorder = recorders_.find(msg->config);

    //if it does not exists make it.
    if(recorder == recorders_.end()) {
        std::shared_ptr<BagRecorder> new_recorder(new BagRecorder(data_folder_));
        recorders_[msg->config] = new_recorder;
    }

    //make sure bag is not active.
    if(recorders_[msg->config]->is_active()) {
        ROS_WARN("Bag configuration %s is already recording to %s.", msg->config.c_str(), recorders_[msg->config]->get_bagname().c_str());
        return;
    }

    //start recording
    std::vector<std::string> topics;
    load_config(msg->config, topics);
    std::string full_bag_name = recorders_[msg->config]->start_recording(msg->bag_name, topics);

    //make sure there were no errors and bag was made
    if(full_bag_name == "") {
        ROS_WARN("Error prevented %s configuration from recording.", msg->config.c_str());
        return;
    }

    //publish bag name
    if(publish_name_) {
        bag_recorder::Rosbag message;
        message.config = msg->config;
        message.bag_name = full_bag_name;
        name_publisher_.publish(message);
    }
    //publish heartbeat
    if(publish_heartbeat_) {
        std_msgs::String message;
        message.data = msg->config;

        //see if it exists already
        std::map<std::string, std::shared_ptr<HeartBeat>>::iterator heartbeat = heartbeats_.find(msg->config);

        //if it doesn't make it
        if(heartbeat == heartbeats_.end()) {
            std::shared_ptr<HeartBeat> beat(new HeartBeat(nh_, heartbeat_topic_, message, heartbeat_interval_));
            heartbeats_[msg->config] = beat;
        }

        //else start it
        heartbeats_[msg->config]->start();
    }

    ROS_INFO("Recording %s configuration to %s.", msg->config.c_str(), full_bag_name.c_str());

    return;
} // Start_Recording()

/**
* @brief Stop_Recording() stops a bag of a certain configuration
* @param [in] msg ROS string msg containing config type to stop
* @details finds recorder in map by config name, stops it
*/
void BagLauncher::Stop_Recording(const std_msgs::String::ConstPtr& msg){
    //find recorder in map
    std::map<std::string, std::shared_ptr<BagRecorder>>::iterator recorder = recorders_.find(msg->data);

    //make sure it exists
    //we do this because a faulty name could be published to us in the message
    //we can't assume we are actually recording msg->data
    if(recorder != recorders_.end()) {
        //stop the bag
        recorder->second->stop_recording();
        ROS_INFO("%s configuration recorder stopped.", msg->data.c_str());
    } else {
        ROS_INFO("%s configuration recorder did not exist.", msg->data.c_str());
    }

    if(publish_heartbeat_) {
        //find heartbeat in map
        std::map<std::string, std::shared_ptr<HeartBeat>>::iterator heartbeat = heartbeats_.find(msg->data);

        //make sure it exists
        if(heartbeat != heartbeats_.end()) {
            //stop it
            heartbeat->second->stop();
        }
    }

    return;
} // Stop_Recording()

/**
* @brief sanitize_topic() makes sure a topic is in proper form
* @param [in] topic name of topic to be sanitized
* @return santized topic name
* @details adds '/' to front of string if not there
*/
std::string BagLauncher::sanitize_topic(std::string topic){
    if(topic.substr(0,1) != "/")
        topic = "/" + topic;
    return topic;
} // sanitize_topic()

/**
* @brief load_config() reads in a .config file
* @param [in] config_name the name of the config file to be loaded (without extension)
* @param [in] topics list of topics to push too
* @param [in] loaded set of configs already read from
* @details reads a config file, parses each line into a vector, sanitizes each.
* allows linking to other config files with '$', also allows comments with '#'
* loads other config files with recursion
*/
void BagLauncher::load_config(std::string config_name, std::vector<std::string>& topics, std::set<std::string> loaded) {
    std::string config_file_name = config_location_ + config_name + ".config";
    std::ifstream fd(config_file_name.c_str());
    std::string line;

    //prevent circular references in config linking
    if(loaded.find(config_name) == loaded.end()) {
        loaded.insert(config_name);
    } else {
        ROS_WARN("%s config loaded alread, circular reference detected.", config_name.c_str());
        return;
    }

    if( !fd ) {
        // if this is the first layer then we record all
        if(loaded.size() <= 1) {
            ROS_ERROR("Topic input file name invalid, recording everything");
            topics.push_back("*");
        } else {
            //else we just throw an error
            ROS_WARN("Linked config: %s is invalid.", config_name.c_str());
        }
    } else {
        while( std::getline(fd,line) ) {
            //ignore blank or lines starting with space or #
            if(line == "" || line.substr(0,1) == " " || line.substr(0,1) == "#")
                continue;
            //link to other config files :P
            //interesting but I doubt it's usefullness
            if(line.substr(0,1) == "$") {
                load_config(line.substr(1), topics, loaded);
                continue;
            }

            topics.push_back(sanitize_topic(line));
        }
    }
} // load_config()

/**
* @brief check_all() is called each loop to beat each of the hearts
* @details iterates over all heartbeats and beats each of them
*/
void BagLauncher::check_all() {
    for(std::map<std::string, std::shared_ptr<HeartBeat>>::iterator it = heartbeats_.begin(); it != heartbeats_.end(); ++it)
        it->second->beat();
} // check_all()

} //bag_launcher_node
