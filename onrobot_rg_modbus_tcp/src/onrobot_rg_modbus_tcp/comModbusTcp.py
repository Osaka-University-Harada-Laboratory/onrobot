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
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection
            dummy (book): the process will be dummy mode (True) or not
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            restartPowerCycle: Restarts the power cycle of Compute Box.
            getStatus: Sends a request to read and returns the gripper status.
    """

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()

    def connectToDevice(self, ip, port, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
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
        """ Closes connection. """

        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        self.client.close()

    def sendCommand(self, message):
        """ Sends a command to the Gripper.

            Args:
                message (list[int]): message to be sent
        """

        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return

        # Sending a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                self.client.write_registers(
                    address=0, values=message, unit=self.changer_addr)

    def restartPowerCycle(self):
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            self.client.write_registers(
                address=0, values=message, unit=restart_address)

    def getStatus(self):
        """ Sends a request to read and returns the gripper status. """

        response = [0] * 18
        if self.dummy:
            rospy.loginfo(
                rospy.get_name() +
                ": " +
                sys._getframe().f_code.co_name)
            return response

        # Getting status from the device (address 258 ~ 275)
        with self.lock:
            response = self.client.read_holding_registers(
                address=258, count=18, unit=self.changer_addr).registers

        # Output the result
        return response
