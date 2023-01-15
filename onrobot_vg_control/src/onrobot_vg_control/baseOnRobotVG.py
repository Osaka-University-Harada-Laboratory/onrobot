#!/usr/bin/env python3

import rospy
from onrobot_vg_control.msg import OnRobotVGInput


class onrobotbaseVG:
    """ onrobotbaseVG sends commands and receives the status of VG gripper.

        Attributes:
            message (list[int]): message including commands to be sent

            verifyCommand:
                Verifies that the value of each variable satisfy its limits.
            refreshCommand:
                Updates the command sent during the next sendCommand() call.
    """

    def __init__(self):
        # Initiating output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """ Verifies that the value of each variable satisfy its limits.

            Args:
                command (OnRobotVGOutput): command message to be verified

            Returns:
                command (OnRobotVGOutput): verified command message
        """

        # Verifying that each variable is in its correct range
        command.rVCA = max(0, command.rVCA)
        command.rVCA = min(255, command.rVCA)
        command.rVCB = max(0, command.rVCB)
        command.rVCB = min(255, command.rVCB)

        # Verifying that the selected mode number is available
        if command.rMCA not in [0x0000, 0x0100, 0x0200]:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the mode number for ch A from" +
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")
        if command.rMCB not in [0x0000, 0x0100, 0x0200]:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the mode number for ch B from" +
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")

        # Returning the modified command
        return command

    def refreshCommand(self, command):
        """ Updates the command sent during the next sendCommand() call.

            Args:
                command (OnRobotVGOutput): command to be refreshed
        """

        # Limiting the value of each variable
        command = self.verifyCommand(command)

        # Initiating command as an empty list
        self.message = []

        # Building the command with each output variable
        self.message.append(command.rMCA)
        self.message.append(command.rVCA)
        self.message.append(command.rMCB)
        self.message.append(command.rVCB)

    def sendCommand(self):
        """ Sends the command to the Gripper. """

        self.client.sendCommand(self.message)

    def getStatus(self):
        """ Requests the gripper status and return OnRobotVGInput message.

            Returns:
                message (list[int]): message including commands to be sent
        """

        # Acquiring status from the Gripper
        status = self.client.getStatus()

        # Messaging to output
        message = OnRobotVGInput()

        # Assignning the values to their respective variables
        message.gVCA = status[0]
        message.gVCB = status[1]

        return message
