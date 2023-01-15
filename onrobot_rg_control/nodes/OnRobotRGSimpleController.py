#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput


def genCommand(char, command):
    """ Updates the command according to the input character.

        Args:
            char (str): set command service request message
            command (OnRobotRGOutput): command to be sent

        Returns:
            command: command message with parameters set
    """

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
        command.rGFR = 400
        command.rGWD = 0
        command.rCTR = 16
    elif char == 'o':
        command.rGFR = 400
        command.rGWD = max_width
        command.rCTR = 16
    elif char == 'i':
        command.rGFR += 25
        command.rGFR = min(max_force, command.rGFR)
        command.rCTR = 16
    elif char == 'd':
        command.rGFR -= 25
        command.rGFR = max(0, command.rGFR)
        command.rCTR = 16
    else:
        # If the command entered is a int, assign this value to rGWD
        try:
            command.rGFR = 400
            command.rGWD = min(max_width, int(char))
            command.rCTR = 16
        except ValueError:
            pass

    return command


def askForCommand(command):
    """ Asks the user for a command to send to the gripper.

        Args:
            command (OnRobotRGOutput): command to be sent

        Returns:
            input(strAskForCommand) (str): input command strings
    """

    currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
    currentCommand += ' rGFR = ' + str(command.rGFR)
    currentCommand += ', rGWD = ' + str(command.rGWD)
    currentCommand += ', rCTR = ' + str(command.rCTR)

    rospy.loginfo(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0 - max width): Go to that position\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """ Main loop which requests new commands and
        publish them on the OnRobotRGOutput topic.
    """

    rospy.init_node(
        'OnRobotRGSimpleController',
        anonymous=True,
        log_level=rospy.DEBUG)
    pub = rospy.Publisher(
        'OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    command = OnRobotRGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg6')
    publisher()
