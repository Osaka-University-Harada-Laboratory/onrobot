#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import rospy
import threading
from pymodbus.client.sync import ModbusTcpClient


class communication:

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()

    def connectToDevice(self, ip, port, changer_addr=65):
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        self.client.connect()

    def disconnectFromDevice(self):
        """Closes connection."""
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client.close()

    def sendCommand(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
        """
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        # Send a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                self.client.write_registers(
                    address=0, values=message, unit=self.changer_addr)

    def restartPowerCycle(self):
        """ Restart Power Cycle of Compute Box
            Necessary is Safety Switch of RG2 or RG6 are pressed
            Writing 2 to this field powers the tool off for a short amount of time and then powers them back
        """
        message = 2
        restart_address = 63
        
        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
                self.client.write_registers(
                    address=0, values=message, unit=restart_address)

    def getStatus(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
        """
        response = [0] * 18
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return response

        # Get status from the device (address 258 ~ 275)
        with self.lock:
            response = self.client.read_holding_registers(
                address=258, count=18, unit=self.changer_addr).registers

        # Output the result
        return response
