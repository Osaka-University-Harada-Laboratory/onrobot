cmake_minimum_required(VERSION 3.0.2)
project(onrobot_vg_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  onrobot_vg_modbus_tcp
)

catkin_python_setup()

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        rospy
        onrobot_vg_modbus_tcp
)

catkin_install_python(
    PROGRAMS nodes/OnRobotVGSimpleController.py nodes/OnRobotVGStatusListener.py
    nodes/OnRobotVGTcpNode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
