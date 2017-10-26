# gameboy_ros

Simple ROS wrapper for the GearBoy Emulator.  Might be useful for deep learning, or something.

Build instructions

1. Clone into a ros workspace with catkin simple
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone --recursive https://github.com/mwatterson/gameboy_ros.git
    git clone https://github.com/catkin/catkin_simple.git
2. Install deps for GearBoy (https://github.com/drhelius/Gearboy)
	cd ..
    sudo rosdep init
    rosdep install --from-paths src --ignore-src -r -y
3. Build using catkin tools (recommended)
	catkin init
	catkin build 
4. Or build using catkin_make
	catkin_init_workspace
	catkin_make


Running instructions

1. Make sure workspace is build and sourced
    source catkin_ws/devel/setup.bash
2. Run Node
	roslaunch gameboy_ros test.launch file:={full path to ROM}
