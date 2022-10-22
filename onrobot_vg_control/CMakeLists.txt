cmake_minimum_required(VERSION 3.0.2)
project(onrobot_vg_control)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  onrobot_vg_modbus_tcp
)

add_message_files(
    FILES
    OnRobotVGInput.msg
    OnRobotVGOutput.msg
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
        onrobot_vg_modbus_tcp
)

catkin_install_python(
    PROGRAMS nodes/OnRobotVGSimpleController.py nodes/OnRobotVGStatusListener.py
    nodes/OnRobotVGTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
