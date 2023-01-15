#!/usr/bin/env python3

import rospy
from onrobot_vg_control.msg import OnRobotVGOutput
from onrobot_vg_control.srv import SetCommand, SetCommandResponse


class OnRobotVGNode:
    """ OnRobotVGNode handles setting commands.

        Attributes:
            pub (rospy.Publisher): the publisher for OnRobotVGOutput
            command (OnRobotVGOutput): command to be sent
            set_command_srv (rospy.Service): set_command service instance

            handleSettingCommand:
                Handles sending commands via socket connection.
            genCommand:
                Updates the command according to the input character.
    """

    def __init__(self):
        self.pub = rospy.Publisher(
            'OnRobotVGOutput', OnRobotVGOutput, queue_size=1)
        self.command = OnRobotVGOutput()
        self.set_command_srv = rospy.Service(
            "/onrobot_vg/set_command",
            SetCommand,
            self.handleSettingCommand)

    def handleSettingCommand(self, req):
        """ Handles sending commands via socket connection. """

        rospy.loginfo(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        rospy.sleep(1)
        return SetCommandResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def genCommand(self, char, command):
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


if __name__ == '__main__':
    rospy.init_node(
        'OnRobotVGSimpleControllerServer',
        anonymous=True,
        log_level=rospy.DEBUG)
    node = OnRobotVGNode()
    rospy.spin()
