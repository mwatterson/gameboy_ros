#include <gameboy_ros/joy_monitor.h>

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
