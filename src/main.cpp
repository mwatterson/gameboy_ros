#include <gameboy_ros/node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gameboy_node");

    GameNode node;
    node.run();

    return 0;
}
