#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput
from onrobot_rg_control.srv import SetCommand, SetCommandResponse


class OnRobotRGDualNode:
    """ OnRobotRGDualNode handles setting commands.

        Attributes:
            pub_primary_gripper (rospy.Publisher):
                the publisher for OnRobotRGOutput for primary gripper A
            pub_secondary_gripper (rospy.Publisher):
                the publisher for OnRobotRGOutput for secondary gripper B
            commandA (OnRobotRGOutput): command for the gripper A to be sent
            commandB (OnRobotRGOutput): command for the gripper B to be sent
            set_command_srv_A (rospy.Service):
                set_command service instance for the gripper A
            set_command_srv_B (rospy.Service):
                set_command service instance for the gripper B

            handleCommandA:
                Handles sending commands for the gripper A.
            handleCommandB:
                Handles sending commands for the gripper B.
            genCommand:
                Updates the command according to the input character.
    """

    def __init__(self):
        self.pub_primary_gripper = rospy.Publisher(
            'OnRobotRGOutput_A', OnRobotRGOutput, queue_size=1)
        self.pub_secondary_gripper = rospy.Publisher(
            'OnRobotRGOutput_B', OnRobotRGOutput, queue_size=1)

        self.commandA = OnRobotRGOutput()
        self.commandB = OnRobotRGOutput()

        self.set_command_srv_A = rospy.Service(
            "/onrobot_rg/set_command_A",
            SetCommand,
            self.handleCommandA)

        self.set_command_srv_B = rospy.Service(
            "/onrobot_rg/set_command_B",
            SetCommand,
            self.handleCommandB)

    def handleCommandA(self, req):
        """ Handles sending commands for the gripper A. """

        rospy.loginfo(str(req.command))
        self.command = self.genCommand(
            str(req.command), self.commandA, gtype=gtype_A)
        self.pub_primary_gripper.publish(self.command)
        rospy.sleep(1)
        return SetCommandResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def handleCommandB(self, req):
        """ Handles sending commands for the gripper B. """

        rospy.loginfo(str(req.command))
        self.command = self.genCommand(
            str(req.command), self.commandB, gtype=gtype_B)
        self.pub_secondary_gripper.publish(self.command)
        rospy.sleep(1)
        return SetCommandResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def genCommand(self, char, command, gtype):
        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotRGOutput): command to be sent
                gtype (str): gripper type 'RG2' or 'RG6'

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
            command.rGFR = max_force
            command.rGWD = 0
            command.rCTR = 16
        elif char == 'o':
            command.rGFR = max_force
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
                command.rGFR = max_force
                command.rGWD = min(max_width, int(char))
                command.rCTR = 16
            except ValueError:
                pass

        return command


if __name__ == '__main__':
    gtype_A = rospy.get_param(
        '/onrobot/gripper_primary', 'rg2')
    gtype_B = rospy.get_param(
        '/onrobot/gripper_secondary', 'rg6')
    rospy.init_node(
        'OnRobotRGDualServer',
        anonymous=True,
        log_level=rospy.DEBUG)
    node = OnRobotRGDualNode()
    rospy.spin()
