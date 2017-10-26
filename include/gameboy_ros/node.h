#ifndef GAMEBOY_ROS_NODE_H_
#define GAMEBOY_ROS_NODE_H_

#include <ros/ros.h>
#include <memory>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <GearboyCore.h>

#include <gameboy_ros/joy_monitor.h>

// Main game node class
class GameNode{
  public:
    GameNode();
    ~GameNode();
    void run();
  private:
    ros::NodeHandle nh_;

    bool color_;
    std::shared_ptr<GearboyCore> emulator_;
    std::vector<GB_Color> pixels_;
    std::vector<uint8_t> encoded_pixels_;
    sensor_msgs::Image img_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    // helper functionspublish_image
    void load();
    void encode_pixels();
    void publish_image();
    JoyMonitor joym_;
    void process_input();
};

#endif // GAMEBOY_ROS_NODE_H_