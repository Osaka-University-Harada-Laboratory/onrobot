#!/usr/bin/env python3

import rospy
import onrobot_rg_control.baseOnRobotRG
import onrobot_rg_modbus_tcp.comModbusTcp
from std_srvs.srv import Trigger, TriggerResponse
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput


class OnRobotRGTcp:
    """ OnRobotRGTcp connects to the gripper with Modbus/TCP.

        Attributes:
            gripper (onrobot_rg_control.baseOnRobotRG.onrobotbaseRG):
                instance of onrobotbaseRG used for the connection establishment
            pub (rospy.Publisher): the publisher for OnRobotRGInput

            restartPowerCycle:
                Restarts the power cycle of the gripper.
            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self):
        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = \
            onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = \
            onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotRGInput'
        self.pub = rospy.Publisher(
            'OnRobotRGInput', OnRobotRGInput, queue_size=1)

        # The Gripper command is received from the topic 'OnRobotRGOutput'
        rospy.Subscriber('OnRobotRGOutput',
                         OnRobotRGOutput,
                         self.gripper.refreshCommand)

        # The restarting service
        rospy.Service(
            "/onrobot_rg/restart_power",
            Trigger,
            self.restartPowerCycle)

        self.mainLoop()

    def restartPowerCycle(self, request):
        """ Restarts the power cycle of the gripper. """

        rospy.loginfo("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg = []
        while not rospy.is_shutdown():
            # Getting and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)

            rospy.sleep(0.05)
            # Sending the most recent command
            if not int(format(status.gSTA, '016b')[-1]):  # not busy
                if not prev_msg == self.gripper.message:  # find new message
                    rospy.loginfo(rospy.get_name()+": Sending message.")
                    self.gripper.sendCommand()
            prev_msg = self.gripper.message
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        gtype = rospy.get_param('/onrobot/gripper', 'rg6')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node(
            'OnRobotRGTcpNode', anonymous=True, log_level=rospy.DEBUG)
        OnRobotRGTcp()
    except rospy.ROSInterruptException:
        pass
