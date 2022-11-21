#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGInput

class DualGripperListen:
    def __init__(self):
        self.statusA = OnRobotRGInput
        self.statusB = OnRobotRGInput

        #Subscribers
        rospy.Subscriber("OnRobotRG_Input_A", OnRobotRGInput, self.Status_A_callback)
        rospy.Subscriber("OnRobotRG_Input_B", OnRobotRGInput, self.Status_B_callback)
        
    
    def Status_A_callback(self, status):
        self.statusA = status

    def Status_B_callback(self, status):
        self.statusB = status

    def status_interpreter(self):

         while not rospy.is_shutdown():
            output = '\n-----\nOnRobot Dual Changer RG status interpreter\n-----\n'
            output += 'Gripper DUAL CHANGER STATUS \n '
            output += '\n ------PRIMARY------ \n'
            output += 'Status_A = ' + str(self.statusA.gSTA) + '\n'
            output += ' Width_A = ' + \
                        str(self.statusA.gWDF) + ' mm\n'
            output += 'Offset_A = ' + \
                        str(self.statusA.gFOF) + ' mm\n'

            output += '\n------SECONDARY------ \n'
            output += 'Status_B = ' + str(self.statusB.gSTA) +'\n'
            output += ' Width_B = ' + \
                        str(self.statusB.gWDF) + ' mm\n'
            output += 'Offset_B = ' + \
                        str(self.statusA.gFOF) + ' mm\n'

            rospy.loginfo(output)

if __name__ == '__main__':
    try:
        rospy.init_node('OnRobotRGStatusDualListen')

        rate = rospy.Rate(20) # 20Hz
        Listener = DualGripperListen()
        Listener.status_interpreter()
    except rospy.ROSInterruptException:
        pass
