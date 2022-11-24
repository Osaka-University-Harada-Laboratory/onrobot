#!/usr/bin/env python3

import rospy
import onrobot_rg_modbus_tcp.comModbusTcp
import onrobot_rg_control.baseOnRobotRG

def mainLoop():
    # Gripper is a RG gripper with a Modbus/TCP connection
    gripper = onrobot_rg_control.baseOnRobotRG
    gripper.client = onrobot_rg_modbus_tcp.comModbusTcp.communication()
    
    #Initialize connection to Compute Box
    restart_address = 63 #0x3F
    gripper.client.connectToDevice(ip, port, restart_address)
    gripper.client.restartPowerCycle()

if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        mainLoop()
    except rospy.ROSInterruptException:
        pass
