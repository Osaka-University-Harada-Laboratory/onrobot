#!/usr/bin/env python3

import rospy
import onrobot_rg_modbus_tcp.comModbusTcp
import onrobot_rg_control.baseOnRobotRG
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput

def mainLoop():
    #Daual quicker changer addresses for primary and secondary side using ModBus/TCP
    primary_address = 66 #0x42
    secondary_address = 67 #0x43

    # Primary side Gripper on Dual Changer Connection
    gripper_primary = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_prime)
    gripper_primary.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)
    gripper_primary.client.connectToDevice(ip, port, primary_address)

    # Secondary side Gripper on Dual changer Connection
    gripper_secondary = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype_second)
    gripper_secondary.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)
    gripper_secondary.client.connectToDevice(ip, port, secondary_address)

    #Initialize Node
    rospy.init_node(
        'OnRobotRGTcpDual', anonymous=True, log_level=rospy.DEBUG)

    #Grippers Status publish
    pub_primary_gripper = rospy.Publisher('OnRobotRG_Input_A', OnRobotRGInput, queue_size=1)
    pub_secondary_gripper = rospy.Publisher('OnRobotRG_Input_B', OnRobotRGInput, queue_size=1)

    #Gripper Commandds reception
    rospy.Subscriber('OnRobotRG_Output_A', OnRobotRGOutput, gripper_primary.refreshCommand)
    rospy.Subscriber('OnRobotRG_Output_B', OnRobotRGOutput, gripper_secondary.refreshCommand)

    #Loop 
    prev_msg_prime = []
    prev_msg_second = []
    while not rospy.is_shutdown():
        #Get Grippers Status
        status_primary = gripper_primary.getStatus()
        status_secondary = gripper_secondary.getStatus()
        #Publish Status
        pub_primary_gripper.publish(status_primary)
        pub_secondary_gripper.publish(status_secondary)

        rospy.sleep(0.05)
        #Update Command primary side
        if not int(format(status_primary.gSTA, '016b')[-1]): #If not busy
            if not prev_msg_prime == gripper_primary.message: #Get new messages
                rospy.loginfo(rospy.get_name()+": Sending Message A Side")
                gripper_primary.sendCommand()
            prev_msg_prime = gripper_primary.message
        
        #Update Command secondary side
        if not int(format(status_secondary.gSTA, '016b')[-1]): #If not busy
            if not prev_msg_second == gripper_secondary.message: #Get new messages
                rospy.loginfo(rospy.get_name()+": Sending Message B Side")
                gripper_secondary.sendCommand()
            prev_msg_second = gripper_secondary.message

            rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        gtype_prime = rospy.get_param('/onrobot/gripper_primary', 'rg2')
        gtype_second = rospy.get_param('/onrobot/gripper_secondary', 'rg6')
        dummy = rospy.get_param('/onrobot/dummy', False)
        mainLoop()
    except rospy.ROSInterruptException:
        pass