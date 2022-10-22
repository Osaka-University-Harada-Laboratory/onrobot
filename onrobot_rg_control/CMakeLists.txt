cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  onrobot_rg_modbus_tcp
  message_generation
)

add_message_files(
    FILES
    OnRobotRGInput.msg
    OnRobotRGOutput.msg
) 

catkin_python_setup()

generate_messages(
    DEPENDENCIES
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    DEPENDS
    CATKIN_DEPENDS
        rospy
        onrobot_rg_modbus_tcp
)

catkin_install_python(
    PROGRAMS nodes/OnRobotRGSimpleController.py nodes/OnRobotRGStatusListener.py
    nodes/OnRobotRGTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
