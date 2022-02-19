cmake_minimum_required(VERSION 3.0.2)
project(onrobot_vg_modbus_tcp)
find_package(catkin REQUIRED COMPONENTS rospy)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS rospy 
)
