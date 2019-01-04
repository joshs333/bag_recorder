# ROS Bag Recorder
## Installing/Running the Recorder
Clone this repository into the src/ folder of a ROS workspace. In the ROS workspace run the following commands.
```
catkin_make
source devel/setup.bash
```
The code should compile. You can then run the node using roslaunch or rosrun.
```
roslaunch bag_recorder default.launch
rosrun bag_recorder bag_recorder_node _configuration_directory:=<path to config> _data_directory:=<path to record bags to>
```
Note that the node requires you to specify configuration_directory and data_directory. Please read below to understand what those are.

## Launch File Usage
To use this ROS bag recorder you need to modify the launch file to fit your project. Especially the data_directory option which is where the bag will be stored.
```
<param name="data_directory"            type="string"       value="$(env HOME)/Desktop/"/>
```
By default the configuration files are in the config folder of the rosbag_recorder package, but you can make it wherever you want.
```
<param name="configuration_directory"   type="string"       value="$(find rosbag_recorder)/config"/>
```
If neither of these parameters are set then the node will not run.

## Configuration File Usage
### Basic Usage
A configuration file is identified by it's name, which is then followed by ".config". If you wanted a configuration called "standard" you would make a file "standard.config" in your configuration folder. In this document you can list out all the topics you want the rosbag_recorder subscribed to.
Here is an example *standard.config*
```
/topic1
/topic2/subtopic1
/topic2/subtopic2
/topic3
/topic4
```

### Subscribe to All Topics
If you wanted this "standard" configuration to subscribe to all topics it could be the following:
```
*
```
A single line containing an asterisk. Note that current behavior also subscribes to all topics when the config file doesn't exist or is empty.

### Comments
You can write comments in the config file a few ways. With a '#' or a space at the beginning of the line. The parser ignores any blank lines or ones starting with ' ' or '#'.
```
# This is a comment
 This will also be ignored by the parser.
/This/will/be/subscribed/to
```
### Config Linking
Say you have your standard config file that is the following.
```
# This is normal
/topica
/topicb
/topicc
```
Then you want to make a new special configuration (let's call it *special*) that is all the topics in the standard config, plus one. You can define it as the following.
```
# This is special
/extra/topic
# Include standard topics
$standard
```
This will link in all the topics in standard.config in addition to any topics in special.config.
The parser will recursively load any linked configs, and will stop when it reaches a circular reference.



## Starting the Recorder
The start topic is set in the launch file as well.
```
<param name="start_bag_topic"           type="string"       value="/record/start"/>
```
You can start the recorder by publishing to this topic with the rosbag message type. This message type has two key values, config and bag_name. Config is the name of the configuration you want the bag launcher to load and subscribe to. To load the config defined above (standard.config) you would set the config value to "standard". The bag_name value is the base name for the bag that will be recorded to. The software is also set up to append the timestamp to this bag name.

This command looks like:
```
rostopic pub /record/start robot_config/rosbag "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
config: 'standard'
bag_name: 'test_bag'"
```
To get this you can type "rostopic pub /record/start " then hit tab a few times until you see the config and bag_name values which you can they go back and edit in command line. If you have the param *start_bag_topic* set to anything else, substitute that in for /record/start.


To see what the full bag name is you can use the bag name publisher. To do this use the following launch options:
```
<param name="publish_name"              type="bool"         value="true"/>
<param name="name_topic"                type="string"       value="/record/bag_name"/>
```
This will publish the full bag name to name_topic.

To see this data you must echo this topic in another terminal window before the start command is sent. This command looks like:
```
rostopic echo /record/bag_name
```
It will print the bag name in that terminal window after the bag is started.

### Starting the recorder programmatically (C++)
The big advantage of this recorder is being programmatic so it can be triggered by a ROS topic. You can do this from any node. To do this you must set bag_recorder and std_msgs as dependencies to use the Rosbag/String message type.
If you make the package using the catkin_create_pkg command you can add it to the list of dependencies.
```
catkin_create_pkg node_name_here roscpp std_msgs bag_recorder other_dependencies
```
If the node is already made you can do the following:
In the CMakeLists.txt add "bag_recorder" and "std_msgs" to the find_package macro.
```
find_package(catkin REQUIRED COMPONENTS
  ...
  std_msgs
  bag_recorder
  ...
)
```
If you use the catkin_package macro you can add "bag_recorder" and "std_msgs" to the CATKIN_DEPENDS category
```
catkin_package(
  INCLUDE_DIRS ...
  LIBRARIES ...
  CATKIN_DEPENDS ... bag_recorder std_msgs ...
  DEPENDS ...
)
```
You must also add bag_recorder and std_msgs to package.xml.
```
  ...
  <build_depend>bag_recorder</build_depend>
  <build_depend>std_msgs</build_depend>
  ...
  <build_export_depend>bag_recorder</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  ...
  <exec_depend>bag_recorder</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  ...
```
This is a bare-bones ROS node that publishes to a start topic.
```
#include <ros/ros.h>
#include <bag_recorder/Rosbag.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");
    ros::Publisher bag_pub = nh.advertise<bag_recorder::Rosbag>("/record/start", 10);

    bag_recorder::Rosbag message;
    message.header.stamp = ros::Time::now();
    message.config = "standard";
    message.bag_name = "Test_bag_name";

    while(ros::ok()) {
        ROS_INFO("Publishing...");
        bag_pub.publish(message);
        ros::spinOnce();
    }

    return 0;
}
```
Read above to understand what the **config** and **bag_name** categories mean.

## Stopping the recorder
The stop topic is defined in the launch file as well.
```
<param name="stop_bag_topic"            type="string"       value="/record/stop"/>
```
To stop a recording of a certain configuration you can publish that configuration to stop_bag_topic. This topic is of std_msgs::String type. Set this messages data value to the configuration you want to stop recording. Continuing the example with "standard.config" you would set data to standard which would then stop the bag recording.

This command looks like:
```
rostopic pub /record/stop std_msgs/String "data: 'standard'"
```
To get this you can type "rostopic pub /record/stop " then hit tab a few times which will fill in the rest, then you can edit the data value. If the param *stop_bag_topic* is set to anything else just substitute that in instead of /record/stop.

### Stopping the recorder programmatically (C++)
Follow the instructions above in Starting the recorder programmatically for adding the "bag_recorder" and "std_msgs" dependencies. The following is a basic ROS node in c++ that will publish a stop command.
```
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");
    ros::Publisher bag_pub = nh.advertise<std_msgs::String>("/record/stop", 10);

    std_msgs::String message;
    message.data = "standard";

    while(ros::ok()) {
        ROS_INFO("Publishing...");
        bag_pub.publish(message);
        ros::spinOnce();
    }

    return 0;
}
```
## Notes
Multiple bags can record at once. However, only one of each configuration can record at once.
If you give it a configuration that does not exist then it will record all topics to the bag.

I favor C++ so I did my sample code above in C++... if you are confused and need sample code in python shoot me an email and I'll throw it together for you.

## Dependancies
This uses only standard ROS/BOOST/STD libraries which should be installed with a standard ROS package.

With this you should be all good!

*Last modified: 01/02/2019*
*joshs333@live.com*
