#!/usr/bin/env python3

import rospy
import onrobot_vg_control.baseOnRobotVG
import onrobot_vg_modbus_tcp.comModbusTcp
from onrobot_vg_control.msg import OnRobotVGInput
from onrobot_vg_control.msg import OnRobotVGOutput


class OnRobotVGTcp:
    """ OnRobotVGTcp connects to the gripper with Modbus/TCP.

        Attributes:
            gripper (onrobot_vg_control.baseOnRobotVG.onrobotbaseVG):
                instance of onrobotbaseVG used for the connection establishment
            pub (rospy.Publisher): the publisher for OnRobotVGInput

            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self):
        # Gripper is a VG gripper with a Modbus/TCP connection
        self.gripper = \
            onrobot_vg_control.baseOnRobotVG.onrobotbaseVG()
        self.gripper.client = \
            onrobot_vg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotVGInput'
        self.pub = rospy.Publisher(
            'OnRobotVGInput', OnRobotVGInput, queue_size=1)

        # The Gripper command is received from the topic 'OnRobotVGOutput'
        rospy.Subscriber('OnRobotVGOutput',
                         OnRobotVGOutput,
                         self.gripper.refreshCommand)

        self.mainLoop()

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg = []
        while not rospy.is_shutdown():
            # Get and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)

            rospy.sleep(0.05)
            # Send the most recent command
            if not prev_msg == self.gripper.message:  # find new message
                rospy.loginfo(rospy.get_name()+": Sending message.")
                self.gripper.sendCommand()
            prev_msg = self.gripper.message
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node(
            'OnRobotVGTcpNode', anonymous=True, log_level=rospy.DEBUG)
        OnRobotVGTcp()
    except rospy.ROSInterruptException:
        pass
