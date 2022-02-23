#!/usr/bin/env python3

import rospy
from onrobot_rg_msgs.msg import OnRobotRGInput


class onrobotbaseRG:
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot RG gripper.
    """

    def __init__(self, gtype):
        # Initiate output message as an empty list
        self.gtype = gtype
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the gripper type from rg2 or rg6.")

        command.rGFR = max(0, command.rGFR)
        command.rGFR = min(max_force, command.rGFR)
        command.rGWD = max(0, command.rGWD)
        command.rGWD = min(max_width, command.rGWD)

        # Verify that the selected mode number is available
        if command.rCTR not in [1, 8, 16]:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the mode number from" +
                "1 (grip), 8 (stop), or 16 (grip_w_offset).")

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
        self.message.append(command.rGFR)
        self.message.append(command.rGWD)
        self.message.append(command.rCTR)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotRGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotRGInput()

        # Assign the values to their respective variables
        message.gFOF = status[0]
        message.gGWD = status[9]
        message.gSTA = status[10]
        message.gWDF = status[17]

        return message
