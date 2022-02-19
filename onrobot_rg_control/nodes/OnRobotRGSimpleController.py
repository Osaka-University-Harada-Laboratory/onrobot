#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGOutput


def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if gtype == 'rg2':
        max_force = 400
        max_width = 1100
    elif gtype == 'rg6':
        max_force = 1200
        max_width = 1600
    else:
        rospy.signal_shutdown(
            rospy.get_name() +
            ": Select the gripper type from rg2 or rg6.")

    if char == 'c':
        command.rGWD = 0
    if char == 'o':
        command.rGWD = max_width

    # If the command entered is a int, assign this value to rGWD
    try:
        command.rGWD = min(max_width, int(char))
    except ValueError:
        pass

    if char == 'i':
        command.rGFR += 25
        command.rGFR = min(max_force, command.rGFR)
    if char == 'd':
        command.rGFR -= 25
        command.rGFR = max(0, command.rGFR)

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
    currentCommand += ' rGFR = ' + str(command.rGFR)
    currentCommand += ', rGWD = ' + str(command.rGWD)
    currentCommand += ', rCTR = ' + str(command.rCTR)

    print(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0 - max width): Go to that position\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and
       publish them on the OnRobotRGOutput topic.
    """

    rospy.init_node('OnRobotRGSimpleController')
    pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput)
    command = OnRobotRGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg6')
    publisher()
