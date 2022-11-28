#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGInput

class OnRobotDualRGStatusListener:
    def __init__(self):
        self.statusA = OnRobotRGInput
        self.statusB = OnRobotRGInput
        self.state= ''

        # Subscribers
        rospy.Subscriber("OnRobotRGInput_A", OnRobotRGInput, self.status_A_callback)
        rospy.Subscriber("OnRobotRGInput_B", OnRobotRGInput, self.status_B_callback)
        
    def status_A_callback(self, status):
        self.statusA = status
        self.statusA.gWDF = status.gWDF / 10.0 # mm 
        self.status_handler(gripper=1)

    def status_B_callback(self, status):
        self.statusB = status
        self.statusB.gWDF = status.gWDF / 10.0 # mm
        self.status_handler(gripper=2)
        
    def status_handler(self, gripper=0):
        output = '\n'
        status = OnRobotRGInput()
        if gripper==1: 
            status = self.statusA
            output+='Primary Gripper current state\n'
        elif gripper==2:
            status= self.statusB
            output+='Secondary Gripper current state\n'
        else:
            return

        gSTA16bit = format(status.gSTA, '016b')
        output += '(gSTA (16 bit) = ' + gSTA16bit + '), Currtent states: \n'
        if int(gSTA16bit[-1]):
            output += ' A motion is ongoing so new commands are not accepted.'
        elif int(gSTA16bit[-2]):
            output += ' An internal- or external grip is detected.'
        elif int(gSTA16bit[-3]):
            output += ' Safety switch 1 is pushed.'
        elif int(gSTA16bit[-4]):
            output += ' Safety circuit 1 is activated so the gripper cannot move.'
        elif int(gSTA16bit[-5]):
            output += ' Safety switch 2 is pushed.'
        elif int(gSTA16bit[-6]):
            output += ' Safety circuit 2 is activated so the gripper cannot move.'
        elif int(gSTA16bit[-7]):
            output += ' Any of the safety switch is pushed.'
        else:
            output ='\n'
        self.state= output

    def status_interpreter(self):
         while not rospy.is_shutdown():
            output = '\n-----\nOnRobot Dual Changer RG status interpreter\n-----\n'
            output += 'Gripper DUAL CHANGER STATUS \n '
            output += '\n ------PRIMARY------ \n'
            output += 'Status_A = ' + str(self.statusA.gSTA) + '\n'
            output += self.state 
            output += 'Width_A = ' + \
                        str(self.statusA.gWDF) + ' mm\n'
            output += 'Offset_A = ' + \
                        str(self.statusA.gFOF) + ' mm\n'
            output += '\n------SECONDARY------ \n'
            output += 'Status_B = ' + str(self.statusB.gSTA) +'\n'
            output += self.state
            output += 'Width_B = ' + \
                        str(self.statusB.gWDF) + ' mm\n'
            output += 'Offset_B = ' + \
                        str(self.statusA.gFOF) + ' mm\n'
            rospy.loginfo(output)


if __name__ == '__main__':
    try:
        rospy.init_node(
            'OnRobotDualRGStatusListener', anonymous=True, log_level=rospy.DEBUG)
        rate = rospy.Rate(20)  # 20 Hz
        Listener = OnRobotDualRGStatusListener()
        Listener.status_interpreter()
    except rospy.ROSInterruptException:
        pass
