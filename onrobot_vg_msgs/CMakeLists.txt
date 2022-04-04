cmake_minimum_required(VERSION 3.0.2)
project(onrobot_vg_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
    FILES
    OnRobotVGInput.msg
    OnRobotVGOutput.msg
) 

generate_messages(
    DEPENDENCIES
)

catkin_package(
    DEPENDS
)
