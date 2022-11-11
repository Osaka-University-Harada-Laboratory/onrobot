#!/usr/bin/env python3

import rospy
import onrobot_vg_modbus_tcp.comModbusTcp
import onrobot_vg_control.baseOnRobotVG
from onrobot_vg_control.msg import OnRobotVGInput
from onrobot_vg_control.msg import OnRobotVGOutput


def mainLoop():
    # Gripper is a VG gripper with a Modbus/TCP connection
    gripper = onrobot_vg_control.baseOnRobotVG.onrobotbaseVG()
    gripper.client = onrobot_vg_modbus_tcp.comModbusTcp.communication(dummy)

    # Connects to the ip address received as an argument
    gripper.client.connectToDevice(ip, port, changer_addr)
    rospy.init_node(
        'OnRobotVGTcpNode', anonymous=True, log_level=rospy.DEBUG)

    # The Gripper status is published on the topic named 'OnRobotVGInput'
    pub = rospy.Publisher('OnRobotVGInput', OnRobotVGInput, queue_size=1)

    # The Gripper command is received from the topic named 'OnRobotVGOutput'
    rospy.Subscriber('OnRobotVGOutput',
                     OnRobotVGOutput,
                     gripper.refreshCommand)

    # We loop
    prev_msg = []
    while not rospy.is_shutdown():
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        rospy.sleep(0.05)
        # Send the most recent command
        if not prev_msg == gripper.message:  # find new message
            rospy.loginfo(rospy.get_name()+": Sending message.")
            gripper.sendCommand()
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        mainLoop()
    except rospy.ROSInterruptException:
        pass
