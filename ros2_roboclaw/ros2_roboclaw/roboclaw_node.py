import rclpy
from rclpy.node import Node

from roboclaw_comm.msg import Speed
from src.roboclaw_driver import SerialPort, Roboclaw
from geometry_msgs.msg import Twist

import logging
import logging.config
import sys
import threading
import traceback
import serial
import os
import time
import math
import sys

class Roboclaw_Node(Node):

    def __init__(self):
        super().__init__('roboclaw_node')
        self.MAX_SPEED = 1200
        self.FRONT_RC_ADDRESS = 129
        self.REAR_RC_ADDRESS = 128
        self.SERIAL_PORT = "/dev/serial1"
        self.BAUD_RATE = 38400
        self.TIMEOUT = 0.7
        self.serial_port = SerialPort(serial.Serial(port=self.SERIAL_PORT, baudrate=self.BAUD_RATE, timeout=self.TIMEOUT))
        self.rc_front = Roboclaw(self.serial_port, self.FRONT_RC_ADDRESS)
        self.rc_rear = Roboclaw(self.serial_port, self.REAR_RC_ADDRESS)

        self.subscription = self.create_subscription(
            Speed,
            'manual',
            self.manual_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.nav_callback,
            10)
        self.subscription

    def manual_callback(self, msg):
        speeds = msg.speed
        if speeds[0] >= 0:
            self.rc_front.drive_forward_m1(speeds[0])
        else:
            self.rc_front.drive_backwards_m1(speeds[0])
        if speeds[1] >= 0:
            self.rc_front.drive_forward_m2(speeds[0])
        else:
            self.rc_front.drive_backwards_m2(speeds[0])
        if speeds[2] >= 0:
            self.rc_rear.drive_forward_m1(speeds[0])
        else:
            self.rc_rear.drive_backwards_m1(speeds[0])
        if speeds[3] >= 0:
            self.rc_rear.drive_forward_m2(speeds[0])
        else:
            self.rc_rear.drive_backwards_m2(speeds[0])

    def nav_callback(self, msg):
        if msg.speed == 0:
            test()
        else:
            self.get_logger().info('I heard: "%d"' % msg.speed)


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = Roboclaw_Node()

    rclpy.spin(roboclaw_node)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()