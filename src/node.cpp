
#include <gameboy_ros/node.h>
#include <sensor_msgs/image_encodings.h>
#include <gearboy.h>

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
