#!/usr/bin/env python3

import rospy
from onrobot_vg_msgs.msg import OnRobotVGInput


class onrobotbaseVG:
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot VG gripper.
    """

    def __init__(self):
        # Initiate output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        command.rVCA = max(0, command.rVCA)
        command.rVCA = min(255, command.rVCA)
        command.rVCB = max(0, command.rVCB)
        command.rVCB = min(255, command.rVCB)

        # Verify that the selected mode number is available
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

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
           during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.rMCA)
        self.message.append(command.rVCA)
        self.message.append(command.rMCB)
        self.message.append(command.rVCB)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotVGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotVGInput()

        # Assign the values to their respective variables
        message.gVCA = status[0]
        message.gVCB = status[1]

        return message
