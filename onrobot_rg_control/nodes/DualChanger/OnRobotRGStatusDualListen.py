#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGInput

def Status_A_callback(status):
    statusA.gFOF = status.gFOF
    statusA.gGWD = status.gGWD
    statusA.gSTA = status.gSTA
    statusA.gWDF = status.gWDF

def Status_B_callback(status):
    statusB.gFOF = status.gFOF
    statusB.gGWD = status.gGWD
    statusB.gSTA = status.gSTA
    statusB.gWDF = status.gWDF

def status_interpreter():
    output = '\n-----\nOnRobot Dual Changer RG status interpreter\n-----\n'
    output += 'Gripper STATUS \n PRIMARY | SECONDARY\n'
    output += 'status_A = ' + str(statusA.gSTA) + '   status_B = ' + str(statusA.gSTA) + '\n'
    
    output += 'Current width between the gripper fingers (w/o offset): ' + \
              str(statusA.gGWD / 10.0) + ' mm\n' + '  ' + str(statusB.gGWD / 10.0) + ' mm\n'

def OnRobotRGStatusListener():
    """Initializes the node and subscribe to both grippers OnRobotRGInput topics."""

    rospy.init_node(
        'OnRobotRGStatusDualListen', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("OnRobotRGInput_A", OnRobotRGInput, Status_A_callback)
    rospy.Subscriber("OnRobotRGInput_B", OnRobotRGInput, Status_B_callback)

    rospy.loginfo(status_interpreter())

    rospy.spin()

if __name__ == '__main__':
    statusA = OnRobotRGInput  
    statusB = OnRobotRGInput

    OnRobotRGStatusListener()
