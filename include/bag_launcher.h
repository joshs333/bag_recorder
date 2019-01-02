/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Joshua Spisak
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/
/// @file bag_launcher.h
/// @author joshs333@live.com
/// @details defines BagLauncher class and includes all headers needed for functions
#pragma once

//ROS Includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bag_recorder/Rosbag.h>

//Bag Recorder
#include <bag_recorder/bag_recorder.h>

//HeartBeat
#include <heartbeat.h>

//STD
#include <string>
#include <vector>

namespace bag_launcher_node {

    using namespace bag_recorder;

    struct BLOptions {
        //initializes a struct of BagLauncher options with default values
        BLOptions();

        //! location to read config files from
        std::string configuration_directory;
        //! location to record bags to
        std::string data_directory;
        //! topic to listen to for start commands
        std::string record_start_topic;
        //! topic to listen to for stop commands
        std::string record_stop_topic;
        //! boolean whether or not to publish the bag name
        bool publish_name;
        //! if publish_name, topic to publish name to
        std::string name_topic;
        //! boolean whether or not to publish a heartbeat that the bag is recording
        bool publish_heartbeat;
        //! if publish_heartbeat, topic to publish a heartbeat to
        std::string heartbeat_topic;
        //! if publish_heartbeat, interval in seconds on which to publish the heartbeat
        double heartbeat_interval;
    };

    class BagLauncher {
        public:
            BagLauncher(ros::NodeHandle nh, BLOptions options);
            ~BagLauncher();

            void check_all();

        private:
            void Start_Recording(const bag_recorder::Rosbag::ConstPtr& msg);
            void Stop_Recording(const std_msgs::String::ConstPtr& msg);
            std::string sanitize_topic(std::string topic);
            void load_config(std::string config_file_name, std::vector<std::string>& topics, std::set<std::string> loaded = std::set<std::string>());

        private:
            ros::NodeHandle nh_;
            std::string config_location_;
            std::string data_folder_;
            ros::Subscriber record_start_subscriber_;
            ros::Subscriber record_stop_subscriber_;
            bool publish_name_;
            bool publish_heartbeat_;
            std::string heartbeat_topic_;
            double heartbeat_interval_;
            ros::Publisher name_publisher_;

            std::map<std::string, std::shared_ptr<HeartBeat>> heartbeats_;
            std::map<std::string, std::shared_ptr<BagRecorder>> recorders_;
    }; //BagLauncher

} // bag_launcher_node
