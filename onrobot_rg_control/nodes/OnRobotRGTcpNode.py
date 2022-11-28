#!/usr/bin/env python3

import rospy
import onrobot_rg_modbus_tcp.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput
from std_srvs.srv import Trigger, TriggerResponse


class OnRobotRGTcp:
    def __init__(self):
        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connects to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic named 'OnRobotRGInput'
        self.pub = rospy.Publisher('OnRobotRGInput', OnRobotRGInput, queue_size=1)

        # The Gripper command is received from the topic named 'OnRobotRGOutput'
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
        rospy.loginfo("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def mainLoop(self):
        prev_msg = []
        while not rospy.is_shutdown():
            # Get and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)

            rospy.sleep(0.05)
            # Send the most recent command
            if not int(format(status.gSTA, '016b')[-1]):  # not busy
                if not prev_msg == self.gripper.message:       # find new message
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
