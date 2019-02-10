/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
/// @file bag_recorder.cpp
/// @brief generates a bag_recorder that subscribes to various topics and
/// records it to a bag
/// @contact joshs333@live.com
/// @details this file is based heavily on the ROS bag recorder class, hence
/// the header above. A few potential race conditions were removed from
/// original modifications and it was also heavily restructured for simplicity
/// and ease of use.
///
/// Original ROSBag Recorder: http://docs.ros.org/diamondback/api/rosbag/html/c++/classrosbag_1_1Recorder.html

//Bulk of includes/ ROS
#include <bag_recorder/bag_recorder.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

// Disk Checking
#include <sys/stat.h>
#include <boost/filesystem.hpp>
// Boost filesystem v3 is default in 1.46.0 and above
// Fallback to original posix code (*nix only) if this is not true
#if BOOST_FILESYSTEM_VERSION < 3
  #include <sys/statvfs.h>
#endif
#if !defined(_MSC_VER)
  #include <termios.h>
  #include <unistd.h>
#endif

// Boost
#include <boost/foreach.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#define foreach BOOST_FOREACH

namespace bag_recorder {

using std::string;
using boost::shared_ptr;
using ros::Time;

OutgoingMessage::OutgoingMessage(string const& _topic, topic_tools::ShapeShifter::ConstPtr _msg, boost::shared_ptr<ros::M_string> _connection_header, Time _time) :
    topic(_topic), msg(_msg), connection_header(_connection_header), time(_time)
{
}

/**
* @brief BagRecorder() Constructor
* @param [in] data_folder directory to record bags into
* @param [in] append_date whether or not to append the date to the bag_name
* @details Initializes basic variables defining recorder behavior
*/
BagRecorder::BagRecorder(std::string data_folder, bool append_date):
    data_folder_(data_folder), append_date_(append_date) {

    ros::NodeHandle nh;

    //ros needs to be working, we check this before we spend a ton of time waiting for a valid time.
    if (!nh.ok())
        return;

    if (!ros::Time::waitForValid(ros::WallDuration(2.0)))
        ROS_WARN("/use_sim_time set to true and no clock published.  Still waiting for valid time...");

    ros::Time::waitForValid();

    // Don't bother doing anything if we never got a valid time
    if (!nh.ok())
        return;
}

/**
* @brief ~BagRecorder() destroys this BagRecorder object
* @details stops recording immediately if currently recording
*/
BagRecorder::~BagRecorder() {
    if(is_active())
        immediate_stop_recording();
    delete message_queue_;
}

/**
* @brief start_recording() starts the bag recorder
* @param [in] bag_name root name of bag to be recorded.
* @param [in] topics vector of topics to be recorded to.
* @return full name of bag that will be recorded to
* @details locks start/stop mutex, generates full bagname, starts bag, starts write thread
*/
std::string BagRecorder::start_recording(std::string bag_name, std::vector<std::string> topics, bool record_all_topics) {
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);

    //will not start new bag_ if there is an active bag_ already
    if(bag_active_)
        return "";
    bag_active_ = true;
    stop_signal_ = false;
    clear_queue_signal_ = false;

    //remove the .bag_ if it is already on the string
    size_t ind = bag_name.rfind(".bag");
    if (ind != std::string::npos && ind == bag_name.size() - 4) {
      bag_name.erase(ind);
    }

    if(append_date_)
        bag_name += string("_") + get_time_str();

    if (bag_name.length() == 0) {
        ROS_ERROR("Bag Name has length 0. Unable to record.");
        return "";
    }

    bag_name += string(".bag");
    bag_filename_ = data_folder_ + bag_name;

    message_queue_ = new std::queue<OutgoingMessage>;

    //test for asterisk to subscribe all topics
    foreach(string const& topic, topics) {
        if(topic.find("*") != std::string::npos) {
            record_all_topics = true;
        }
    }

    if(record_all_topics) {
        recording_all_topics_ = true;
        subscribe_all();
    } else if(topics.size() > 0) {
        // Subscribe to specified topics
        foreach(string const& topic, topics)
            //prevent multiple subscriptions
            if (subscribed_topics_.find(topic) == subscribed_topics_.end()) {
                try {
                    subscribers_.push_back(generate_subscriber(topic));
                } catch(ros::InvalidNameException) {
                    ROS_ERROR("Invalid topic name: %s, no subscriber generated.", topic.c_str());
                }
            }
    } else {
        ROS_ERROR("No Topics Supplied to be recorded. Aborting bag %s.", bag_name.c_str());
        return "";
    }

    // Open bag_ file for writing
    // This got moved here from queue_processor so all the errors that would
    // cause the bag not to start would be in here.
    bag_.setCompression(rosbag::compression::Uncompressed);
    bag_.setChunkThreshold(1024 * 768);

    try {
        bag_.open(bag_filename_ + string(".active"), rosbag::bagmode::Write);
    }
    catch (rosbag::BagException e) {
        //needs the mutex because the start functions read bag_active_
        ROS_ERROR("Error writing: %s", e.what());
        bag_active_ = false;
        return "";
    }

    // start write thread
    record_thread_ = boost::thread(boost::bind(&BagRecorder::queue_processor, this));
    queue_condition_.notify_all();

    return bag_name;
} // start_recording()

/**
* @brief stop_recording() stops the bag recorder but allows the queue to be emptied
* @details gets start/stop mutex, sets stop flag, kills subscribers
*/
void BagRecorder::stop_recording() {
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);

    //if not recording then do nothing
    if(!bag_active_)
        return;

    clear_queue_signal_ = true;

    //note that start_stop_lock is acting as a lock for both subscribers_
    //and also for subscribed_topics_
    foreach( boost::shared_ptr<ros::Subscriber> sub, subscribers_ )
        sub->shutdown();

    subscribed_topics_.clear();

    ROS_INFO("Stopping BagRecorder, clearing queue.");
} // stop_recording()

/**
* @brief immediate_stop_recording() stops the bag immediately
* @details obtains the start/stop mutex to set start/stop veriables. Sets a flag
* such that the queue_processor will stop recording immeidately.
* Also unsubscribes from all topics.
*/
void BagRecorder::immediate_stop_recording() {
    //needs the mutex because the stop signals and subscibers can be accessed
    //by the write thread
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);

    //if not writing then do nothing.
    if(!bag_active_)
        return;

    stop_signal_ = true;

    foreach( boost::shared_ptr<ros::Subscriber> sub, subscribers_ )
        sub->shutdown();

    subscribed_topics_.clear();

    ROS_INFO("Stopping BagRecorder immediately.");
} // immediate_stop_recording()

/**
* @brief is_active() tells if a bag is currently being recorded to
* @param [in] <name> <parameter_description>
* @return true if currently recording, false if not
*/
bool BagRecorder::is_active() {
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);
    return bag_active_;
} // is_active()

/**
* @brief get_bagname() returns the filename of the bag being recorded to
* @return returns bag_filename_
* @details returns the class variable of the bag_filename_ even if not
* currently recording
*/
std::string BagRecorder::get_bagname() {
    return bag_filename_;
}

/**
* @brief can_log() checks class variables to make sure it is safe to write to file
* @return true if safe to write, false if not
* @details basically returnes checks_failed_, also issues an warning every
* 5 seconds if checks_failed_ is false
*/
bool BagRecorder::can_log() {
    if (!checks_failed_)
        return true;

    //sends warning every 5 seconds if this is called continuously
    if (ros::WallTime::now() >= warn_next_) {
        warn_next_ += ros::WallDuration().fromSec(5.0);
        ROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
    }
    return false;
} // can_log()

/**
* @brief is_subscribed_to() sees if the BagRecorder is subscribed to a topic
* @param [in] topic topic to be checked for
* @return true if subscribed, false if not
* @details sees if topic string is located in the set of subscribed_topics_
*/
bool BagRecorder::is_subscribed_to(std::string topic) {
    //all calls to subscribed_topics_ is already protected by the start/stop
    //mutex so I'll just use that here instead of making another call
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);
    return (subscribed_topics_.find(topic) != subscribed_topics_.end());
} // is_subscribed_to()

/**
* @brief generate_subscriber() generates a generic subscriber to any topic.
* @param [in] topic topic that the subscriber will be generated for
* @return returns a boost shared_ptr to a ros subscriber
*/
shared_ptr<ros::Subscriber> BagRecorder::generate_subscriber(string const& topic) {
	ROS_DEBUG("Subscribing to %s", topic.c_str());

    ros::NodeHandle nh;
    shared_ptr<int> count(boost::make_shared<int>(0));
    shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());

    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    //lol what a line of code! #C++templates_rock!
    ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
        const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
            boost::bind(&BagRecorder::subscriber_callback, this, _1, topic, sub, count));
    *sub = nh.subscribe(ops);


    ROS_INFO("Subscribing to topic: %s", topic.c_str());
    subscribed_topics_.insert(topic);

    return sub;
} // generate_subscriber()

/**
* @brief subscriber_callback() takes information from topics it's subscribed to and adds it to the queue
* @param [in] msg_event generic ROS message class
* @param [in] topic topic name in string form
* @param [in] subscriber pointer to the subscriber, fills template requirements but not used
* @param [count] pointer to an in, fills template requirements but not used
* @details turns a message into and OutgoingMessage and adds it to the queue to be written.
*/
void BagRecorder::subscriber_callback(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event, string const& topic, shared_ptr<ros::Subscriber> subscriber, shared_ptr<int> count) {
    //These do nothing, but fill the template requirements
    (void)subscriber;
    (void)count;

    Time rectime = Time::now();

    //generates new outgoing message
    OutgoingMessage out(topic, msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), rectime);

    { //writes message to queue
        boost::mutex::scoped_lock queue_lock(queue_mutex_);
        message_queue_->push(out);
    }

    //notify write thread
    queue_condition_.notify_all();
} // subscriber_callback()

/**
* @brief queue_processor() actually opens the bag file, writes and closes the bag file
* @details Primary loop locks queue to pull outgoing messages to write to the bag.
* Also locks start/stop variables to see if it needs to stop. Runs scheduled checks.
*/
void BagRecorder::queue_processor() {
    ROS_INFO("Recording to %s.", bag_filename_.c_str());

    // schedule checks now that write queue is running
    warn_next_ = ros::WallTime();
    check_disk_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(check_disk_interval_);
    subscribe_all_next_ = ros::WallTime::now() + ros::WallDuration().fromSec(subscribe_all_interval_);
    check_disk();
    //note we do not need to run subscribe all because it was run during the start bag

    // Technically the queue_mutex_ should be locked while checking empty.
    // Except it should only get checked if the node is not ok, and thus
    // it shouldn't be in contention.
    ros::NodeHandle nh;
    while ((nh.ok() || !message_queue_->empty())) {
        boost::unique_lock<boost::mutex> queue_lock(queue_mutex_);

        bool finished = false;
        while (message_queue_->empty()) {
            //will finish if queue is empty and either stop_signal_ is recieved
            {
                boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);
                if(stop_signal_ || clear_queue_signal_) {
                    finished = true;
                    break;
                }
            }

            //will finish if queue is empty and ros has issues
            if (!nh.ok()) {
                queue_lock.release()->unlock();
                finished = true;
                break;
            }

            //even if queue is empty we want to run checks
            //so this way we can subscribe to other topics
            run_scheduled_checks();

            //do not get stuck in loop
            boost::xtime xt;
            #if BOOST_VERSION >= 105000
                boost::xtime_get(&xt, boost::TIME_UTC_);
            #else
                boost::xtime_get(&xt, boost::TIME_UTC);
            #endif
            xt.nsec += 250000000;
            //check every 1/4th second if queue is empty or end condition
            //note that the timed_wait function unlocks queue_lock
            //so other threads can write to the queue
            queue_condition_.timed_wait(queue_lock, xt);
        }
        //if finished flag is set to true stop recording or stop_signal_ recieved
        {
            boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);
            if (finished || stop_signal_)
                break;
        }

        //if we get here queue is not empty, write next message to queue
        OutgoingMessage out = message_queue_->front();
        message_queue_->pop();

        queue_lock.release()->unlock();

        //perform safety checks before writing
        run_scheduled_checks();

        //if checks passed, can log, then log
        if (can_log())
            bag_.write(out.topic, out.time, *out.msg, out.connection_header);
    }

    ROS_INFO("Closing %s.", bag_filename_.c_str());
    bag_.close();
    rename((bag_filename_ + string(".active")).c_str(), bag_filename_.c_str());

    // do not need the mutex because the function killing subscribers has a mutex
    // that holds the queue processor. So when we get here all subcribers are
    // dead and nothing else is using the queue.
    while(!message_queue_->empty()) message_queue_->pop();

    //needs the mutex because the start functions read bag_active_
    boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);
    bag_active_ = false;
} // queue_processor()

/**
* @brief run_scheduled_checks() sees if a function is scheduled to run and runs it if so
* @details runs both check_disk() and subscribe_all() depending on if they are configured to run and scheduled to.
*/
void BagRecorder::run_scheduled_checks() {
    if (ros::WallTime::now() < check_disk_next_) {
        check_disk_next_ += ros::WallDuration().fromSec(check_disk_interval_);
        check_disk();
    }

    //if any of the stop signals were recieved will not run subscribe all
    if(ros::WallTime::now() >= subscribe_all_next_) {
        subscribe_all_next_ += ros::WallDuration().fromSec(subscribe_all_interval_);

        boost::mutex::scoped_lock start_stop_lock(start_stop_mutex_);

        if(!(stop_signal_ || clear_queue_signal_) && recording_all_topics_) {
            subscribe_all();
        }
    }
} // run_scheduled_checks()

/**
* @brief subscribe_all() subscribes to all topics known to master
* @details gets all known topics from master, subscribes if not already subscribed.
*/
void BagRecorder::subscribe_all() {
    //does not call a mutex because all calls to subscribe_all
    //have the mutex already.
    //gets topic info, subscribes to any not already subscribed to
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics)) {
        foreach(ros::master::TopicInfo const& t, topics) {
            //note that this doesn't use is_subscribed_to because
            //is_subscribed_to wants a mutex we already have
            if (subscribed_topics_.find(t.name) == subscribed_topics_.end())
                subscribers_.push_back(generate_subscriber(t.name));
        }
    }
} // subscribe_all()

/**
* @brief check_disk() performs various checks on the disk to make sure it is safe to write
* @details Uses statvfs if BOOST version < 3, else uses boost. Sets class
* variable checks_failed_ to true or false depending on results.
*/
void BagRecorder::check_disk() {
#if BOOST_FILESYSTEM_VERSION < 3
    struct statvfs fiData;
    if ((statvfs(bag_.getFileName().c_str(), &fiData)) < 0)
    {
        ROS_WARN("Failed to check filesystem stats.");
        return;
    }
    unsigned long long free_space = 0;
    free_space = (unsigned long long) (fiData.f_bsize) * (unsigned long long) (fiData.f_bavail);
    if (free_space < min_recording_space_)
    {
        ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.", min_recording_space_str_.c_str(), bag_.getFileName().c_str());
        checks_failed_ = true;
        return;
    }
    else if (free_space < 5 * min_recording_space_)
    {
        ROS_WARN("Less than 5 x %s of space free on disk with %s.", min_recording_space_str_.c_str(), bag_.getFileName().c_str());
    }
    else
    {
        checks_failed_ = false;
    }
#else
    boost::filesystem::path p(boost::filesystem::system_complete(bag_.getFileName().c_str()));
    p = p.parent_path();
    boost::filesystem::space_info info;
    try
    {
        info = boost::filesystem::space(p);
    }
    catch (boost::filesystem::filesystem_error &e)
    {
        ROS_WARN("Failed to check filesystem stats [%s].", e.what());
        checks_failed_ = true;
        return;
    }
    if ( info.available < min_recording_space_)
    {
        ROS_ERROR("Less than %s of space free on disk with %s.  Disabling recording.", min_recording_space_str_.c_str(), bag_.getFileName().c_str());
        checks_failed_ = true;
        return;
    }
    else if (info.available < 5 * min_recording_space_)
    {
        ROS_WARN("Less than 5 x %s of space free on disk with %s.", min_recording_space_str_.c_str(), bag_.getFileName().c_str());
        checks_failed_ = false;
    }
    else
    {
        checks_failed_ = false;
    }
#endif
    return;
} // check_disk()

/**
* @brief get_time_str() returns a timestamp in string form
* @return timestamp in form YYYY-MM-DD-HH-MM-SS
*/
std::string BagRecorder::get_time_str()
{
    std::stringstream msg;
    const boost::posix_time::ptime now=
        boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f=
        new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
} // get_time_str()

} // bag_recorder
