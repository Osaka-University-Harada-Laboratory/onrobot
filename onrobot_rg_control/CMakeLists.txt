cmake_minimum_required(VERSION 3.0.2)
project(onrobot_rg_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  onrobot_rg_modbus_tcp
)

catkin_python_setup()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        rospy
        onrobot_rg_modbus_tcp
)

catkin_install_python(
    PROGRAMS nodes/OnRobotRGSimpleController.py nodes/OnRobotRGStatusListener.py
    nodes/OnRobotRGTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
