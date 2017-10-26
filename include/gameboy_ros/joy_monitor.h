#ifndef GAMEBOY_ROS_JOY_MONITOR_H_
#define GAMEBOY_ROS_JOY_MONITOR_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <definitions.h>

// interface to ros Joy
class JoyMonitor {
public:
    JoyMonitor();
    std::vector<std::pair<Gameboy_Keys,bool> > pop_events();
private:
    void joy_cb(const sensor_msgs::Joy &msg);
    std::vector<int> buttons_;
    std::vector<int> axis_;
    float deadzone_;
    std::map<uint,Gameboy_Keys> button_config_;
    std::map<int,Gameboy_Keys> axis_config_;
    ros::NodeHandle nh_;
    std::vector<std::pair<Gameboy_Keys,bool> > events_; // bool true for pressed, false for released
    ros::Subscriber joy_sub_;
};

#endif // GAMEBOY_ROS_JOY_MONITOR_H_ 