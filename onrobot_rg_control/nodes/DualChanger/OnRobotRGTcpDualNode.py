#!/usr/bin/env python3

import rospy
import onrobot_rg_modbus_tcp.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput
from std_srvs.srv import Trigger, TriggerResponse

class OnRobotDualRGTcp:
    def __init__(self):
        # Daual quicker changer addresses for primary and secondary side using ModBus/TCP
        primary_address = 66 #0x42
        secondary_address = 67 #0x43

        # Primary side Gripper on Dual Changer Connection
        self.gripper_primary = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_prime)
        self.gripper_primary.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)
        self.gripper_primary.client.connectToDevice(ip, port, primary_address)

        # Secondary side Gripper on Dual changer Connection
        self.gripper_secondary = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_second)
        self.gripper_secondary.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)
        self.gripper_secondary.client.connectToDevice(ip, port, secondary_address)

        #Grippers Status publish
        self.pub_primary_gripper = rospy.Publisher('OnRobotRGInput_A', OnRobotRGInput, queue_size=1)
        self.pub_secondary_gripper = rospy.Publisher('OnRobotRGInput_B', OnRobotRGInput, queue_size=1)

        #Gripper Commandds reception
        rospy.Subscriber('OnRobotRGOutput_A', OnRobotRGOutput, self.gripper_primary.refreshCommand)
        rospy.Subscriber('OnRobotRGOutput_B', OnRobotRGOutput, self.gripper_secondary.refreshCommand)

        # The restarting service
        rospy.Service(
            "/onrobot_rg/restart_power",
            Trigger,
            self.restartPowerCycle)

        self.mainLoop()

    def restartPowerCycle(self, request):
        rospy.loginfo("Restart the power cycle of all grippers connected.")
        self.gripper_primary.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def mainLoop(self):
        prev_msg_prime = []
        prev_msg_second = []
        while not rospy.is_shutdown():
            # Get Grippers Status
            status_primary = self.gripper_primary.getStatus()
            status_secondary = self.gripper_secondary.getStatus()
            # Publish Status
            self.pub_primary_gripper.publish(status_primary)
            self.pub_secondary_gripper.publish(status_secondary)

            rospy.sleep(0.05)
            # Update Command primary side
            if not int(format(status_primary.gSTA, '016b')[-1]): #If not busy
                if not prev_msg_prime == self.gripper_primary.message: #Get new messages
                    rospy.loginfo(rospy.get_name()+": Sending Message A Side")
                    self.gripper_primary.sendCommand()
                prev_msg_prime = self.gripper_primary.message
            # Update Command secondary side
            if not int(format(status_secondary.gSTA, '016b')[-1]): #If not busy
                if not prev_msg_second == self.gripper_secondary.message: #Get new messages
                    rospy.loginfo(rospy.get_name()+": Sending Message B Side")
                    self.gripper_secondary.sendCommand()
                prev_msg_second = self.gripper_secondary.message
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        gtype_prime = rospy.get_param('/onrobot/gripper_primary', 'rg2')
        gtype_second = rospy.get_param('/onrobot/gripper_secondary', 'rg6')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node(
            'OnRobotDualRGTcp', anonymous=True, log_level=rospy.DEBUG)
        OnRobotDualRGTcp()
    except rospy.ROSInterruptException:
        pass