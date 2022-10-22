#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput
from onrobot_rg_control.srv import SetCommand, SetCommandResponse


class OnRobotRGNode:
    """Class to handle setting commands."""
    def __init__(self):
        self.pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
        self.command = OnRobotRGOutput()
        self.set_command_srv = rospy.Service(
            "/onrobot_rg/set_command",
            SetCommand,
            self.handle_set_command)

    def handle_set_command(self, req):
        """To handle sending commands via socket connection."""
        rospy.loginfo(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        rospy.sleep(1)
        return SetCommandResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def genCommand(self, char, command):
        """Updates the command according to the character entered by the user."""

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


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg6')
    rospy.init_node('OnRobotRGSimpleControllerServer', log_level=rospy.DEBUG)
    node = OnRobotRGNode()
    rospy.spin()
