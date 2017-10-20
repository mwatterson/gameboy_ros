#include <ros/ros.h>
#include "std_msgs/String.h"
#include <memory>
#include <gearboy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>

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
JoyMonitor::JoyMonitor(): nh_("~") {
    int index;
    if(nh_.getParam("button_mapping/A",index))
        button_config_[uint(index)] = Gameboy_Keys::A_Key;
    if(nh_.getParam("button_mapping/B",index))
        button_config_[uint(index)] = Gameboy_Keys::B_Key;
    if(nh_.getParam("button_mapping/Start",index))
        button_config_[uint(index)] = Gameboy_Keys::Start_Key;
    if(nh_.getParam("button_mapping/Select",index))
        button_config_[uint(index)] = Gameboy_Keys::A_Key;
    if(nh_.getParam("button_mapping/Right",index))
        button_config_[uint(index)] = Gameboy_Keys::Right_Key;
    if(nh_.getParam("button_mapping/Left",index))
        button_config_[uint(index)] = Gameboy_Keys::Left_Key;
    if(nh_.getParam("button_mapping/Up",index))
        button_config_[uint(index)] = Gameboy_Keys::Up_Key;
    if(nh_.getParam("button_mapping/Down",index))
        button_config_[uint(index)] = Gameboy_Keys::Down_Key;
    if(nh_.getParam("button_mapping/Down",index))
        button_config_[uint(index)] = Gameboy_Keys::Down_Key;
    if(nh_.getParam("deadzone",deadzone_))
        deadzone_ = 0.1; // this is okay to be silent
    if(nh_.getParam("axis_mapping/UpDown",index)) {
        axis_config_[index] = Gameboy_Keys::Up_Key;
        axis_config_[-index] = Gameboy_Keys::Down_Key;
    }
    if(nh_.getParam("axis_mapping/LeftRight",index)) {
        axis_config_[index] = Gameboy_Keys::Left_Key;
        axis_config_[-index] = Gameboy_Keys::Right_Key;
    }


    if(button_config_.size()==0 && axis_config_.size() == 0)
        ROS_WARN("Failed to load ros joy button mappings! Input will not work!");


    joy_sub_ = nh_.subscribe("input",10,&JoyMonitor::joy_cb,this);
}
void JoyMonitor::joy_cb(const sensor_msgs::Joy &msg) {
    // threshhold buttons
    std::vector<int> axis_new;
    for(auto &a:msg.axes) {
        if(a > deadzone_)
            axis_new.push_back(1);
        else if(a < -deadzone_)
            axis_new.push_back(-1);
        else
            axis_new.push_back(0);
    }

    if(buttons_.size()==0) {
        buttons_ = msg.buttons;
    }
    if(axis_.size()==0) {
        axis_ = axis_new;
    }
    if(buttons_.size() != msg.buttons.size() || axis_.size() != axis_new.size() ) {
        ROS_ERROR("Size of ros joy msg changed. This should never happen!");
        buttons_ = msg.buttons;
        axis_ = axis_new;
    }
    for(uint i=0; i < msg.buttons.size();i++) {
        if(msg.buttons[i]!=buttons_[i]) {
            auto it = button_config_.find(i);
            if(it != button_config_.end())
                events_.push_back(std::make_pair(it->second,msg.buttons[i] > 0));
        }
    }
    for(uint i=0; i < axis_.size();i++) {
        // treating a continous axis like a discreet one has two cases:
        // 1. we are entering/leaving the deadzone
        // 2. we skipped the deadzone
        if(axis_[i]!=axis_new[i]) {
            if(axis_[i] == 0 || axis_new[i] == 0) {
                int idx = i*(axis_[i]+ axis_new[i]);
                auto it = axis_config_.find(idx);
                if(it != axis_config_.end())
                    events_.push_back(std::make_pair(it->second, axis_new[i] != 0 ));
            } else {
                auto it1 = axis_config_.find(i*axis_[i]);
                auto it2 = axis_config_.find(i*axis_new[i]);
                if(it1 != axis_config_.end() && it2 != axis_config_.end()) {
                    events_.push_back(std::make_pair(it1->second, false ));
                    events_.push_back(std::make_pair(it2->second, true ));
                }

            }
        }
    }
    buttons_ = msg.buttons;
    axis_ = axis_new;
}
std::vector<std::pair<Gameboy_Keys,bool> > JoyMonitor::pop_events() {
    std::vector<std::pair<Gameboy_Keys,bool> > rval;
    rval = events_;
    events_.clear();
    return rval;
}

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

GameNode::GameNode() : nh_("~") , pixels_(GAMEBOY_WIDTH*GAMEBOY_HEIGHT), encoded_pixels_(4*GAMEBOY_WIDTH*GAMEBOY_HEIGHT), it_(nh_){
   emulator_ = std::make_shared<GearboyCore>();
   emulator_->Init();
   img_.data.resize(4*GAMEBOY_WIDTH*GAMEBOY_HEIGHT);
   load();
   image_pub_ = it_.advertise("image_raw",1);
}
GameNode::~GameNode() {
    emulator_->SaveRam();
}

void GameNode::load() {
    std::string file;
    if(!nh_.getParam("file",file)) {
        ROS_FATAL_STREAM("No file given");
        ros::shutdown();
        return;
    }
    // load the file
    emulator_->SaveRam();
    emulator_->LoadROM(file.c_str(),false);
    emulator_->LoadRam();
    if(!emulator_->GetCartridge()->IsValidROM()) {
        ROS_FATAL_STREAM("Failed to load file: " << file << " (relative paths are to ~/.ros)");
        ros::shutdown();
        return;
    }
    color_ = emulator_->GetCartridge()->IsCGB();
    // turn off sound
    emulator_->EnableSound(false);
}
void GameNode::publish_image() {
    img_.step = 4*GAMEBOY_WIDTH;
    img_.encoding  = sensor_msgs::image_encodings::RGBA8;
    img_.height = GAMEBOY_HEIGHT;
    img_.width = GAMEBOY_WIDTH;
    img_.header.frame_id = "gameboy";
    img_.header.stamp = ros::Time::now();
    std::swap(img_.data,encoded_pixels_);
    image_pub_.publish(img_);

}
void GameNode::encode_pixels(){
    int c = 0;
    for(int i=0;i<GAMEBOY_HEIGHT;i++) {
        for(int j=0;j<GAMEBOY_WIDTH;j++) {
            int px = i*GAMEBOY_WIDTH + j;
            encoded_pixels_[c++] = pixels_[px].red;
            encoded_pixels_[c++] = pixels_[px].green;
            encoded_pixels_[c++] = pixels_[px].blue;
            encoded_pixels_[c++] = pixels_[px].alpha;
        }
    }
}

void GameNode::run() {
    ros::Rate r(60);
    while(nh_.ok()) {
        process_input();
        emulator_->RunToVBlank(pixels_.data());

        // publish frame
        encode_pixels();
        publish_image();

        ros::spinOnce();
        r.sleep();
    }
}
void GameNode::process_input() {
    auto events = joym_.pop_events();
    for(auto &e:events) {
        if(e.second)
            emulator_->KeyPressed(e.first);
        else
            emulator_->KeyReleased(e.first);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gameboy_node");

    GameNode node;
    node.run();

    return 0;
}
