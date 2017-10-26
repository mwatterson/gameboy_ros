cmake_minimum_required(VERSION 2.8.3)
project(gameboy_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations  -Werror=return-type")

find_package(catkin_simple REQUIRED)

# Build Gearboy
set(GEARBOY_INCLUDES external/Gearboy/src external/Gearboy/src/audio external/Gearboy/src/miniz)
include_directories(${GEARBOY_INCLUDES})

file(GLOB GEARBOY_MAIN external/Gearboy/src/*.cpp)
file(GLOB GEARBOY_AUDIO external/Gearboy/src/audio/*.cpp)
file(GLOB GEARBOY_MINIZ external/Gearboy/src/miniz/*.cpp)

add_library(gearboy ${GEARBOY_MAIN} ${GEARBOY_AUDIO} ${GEARBOY_MINIZ})
target_link_libraries(gearboy SDL2 GLEW GLU GL)

include_directories(include)

catkin_simple()

# Build ROS Node
cs_add_executable(gameboy_node src/joy_monitor.cpp src/main.cpp src/node.cpp)
target_link_libraries(gameboy_node gearboy)


# Catkin simple stuff
cs_install()

cs_export()
