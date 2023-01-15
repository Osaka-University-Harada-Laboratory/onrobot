#!/usr/bin/env python3

import rospy
from onrobot_vg_control.msg import OnRobotVGOutput


def genCommand(char, command):
    """ Updates the command according to the input character.

        Args:
            char (str): set command service request message
            command (OnRobotVGOutput): command to be sent

        Returns:
            command: command message with parameters set
    """

    if char == 'g':
        command.rMCA = 0x0100
        command.rVCA = 255
        command.rMCB = 0x0100
        command.rVCB = 255
    if char == 'r':
        command.rMCA = 0x0000
        command.rVCA = 0
        command.rMCB = 0x0000
        command.rVCB = 0
    if char == 'ga':
        command.rMCA = 0x0100
        command.rVCA = 255
    if char == 'ra':
        command.rMCA = 0x0000
        command.rVCA = 0
    if char == 'gb':
        command.rMCB = 0x0100
        command.rVCB = 255
    if char == 'rb':
        command.rMCB = 0x0000
        command.rVCB = 0

    # If the command entered is a int, assign this value to r
    try:
        if int(char) == 0:
            command.rMCA = 0x0000
            command.rVCA = 0
            command.rMCB = 0x0000
            command.rVCB = 0
        else:
            command.rMCA = 0x0100
            command.rVCA = min(255, int(char))
            command.rMCB = 0x0100
            command.rVCB = min(255, int(char))
    except ValueError:
        pass

    return command


def askForCommand(command):
    """ Asks the user for a command to send to the gripper.

        Args:
            command (OnRobotVGOutput): command to be sent

        Returns:
            input(strAskForCommand) (str): input command strings
    """

    currentCommand = 'Simple OnRobot VG Controller\n-----\nCurrent command:'
    currentCommand += ' rMCA = ' + str(command.rMCA)
    currentCommand += ', rVCA = ' + str(command.rVCA)
    currentCommand += ', rMCB = ' + str(command.rMCB)
    currentCommand += ', rVCB = ' + str(command.rVCB)

    rospy.loginfo(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'g: Turn on all channels\n'
    strAskForCommand += 'r: Turn off all channels\n'
    strAskForCommand += 'ga: Turn on channel A\n'
    strAskForCommand += 'ra: Turn off channel A\n'
    strAskForCommand += 'gb: Turn on channel B\n'
    strAskForCommand += 'rb: Turn off channel B\n'
    strAskForCommand += '(0 - 255): Set vacuum power for all channels\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """ Main loop which requests new commands and
        publish them on the OnRobotVGOutput topic.
    """

    rospy.init_node(
        'OnRobotVGSimpleController',
        anonymous=True,
        log_level=rospy.DEBUG)
    pub = rospy.Publisher(
        'OnRobotVGOutput', OnRobotVGOutput, queue_size=1)
    command = OnRobotVGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    publisher()
