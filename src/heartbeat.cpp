/// @file heartbeat.cpp
/// @author joshs333@live.com
/// @details this is a simple class to publish a string every N seconds
#include "heartbeat.h"

/**
* @brief HeartBeat() Contructor initializes class variables
* @param [in] nh ROS nodehandle to use
* @param [in] topic topic to publish heartbeat to
* @param [in] message string to publish on topic
* @param [in] interval seconds between each heartbeat
*/
HeartBeat::HeartBeat(ros::NodeHandle nh, std::string topic, std_msgs::String message, double interval):
  heartbeat_publisher_(nh.advertise<std_msgs::String>(topic, 10)), message_(message), interval_(ros::Duration().fromSec(interval)), beat_(false) {
}

/**
* @brief start() makes the HeartBeat beat.
* @details sets beat_ to true, schedules the next heartbeat
*/
void HeartBeat::start() {
    next_beat_ = ros::Time::now();
    beat_ = true;

    beat();
}

/**
* @brief stop() makes the heart stop
* @details sets beat_ to false
*/
void HeartBeat::stop() {
    beat_ = false;
}

/**
* @brief beat() publishes the heartbeat
* @details beats if beat_ is true and is scheduled.
*/
void HeartBeat::beat() {
    //return if not scheduled or enabled
    if(!beat_ || ros::Time::now() < next_beat_)
        return;

    //schedule next heartbeat
    next_beat_ += interval_;

    //publish heartbeat
    heartbeat_publisher_.publish(message_);
}
