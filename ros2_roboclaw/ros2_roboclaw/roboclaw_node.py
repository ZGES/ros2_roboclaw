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
        self.MAX_SPEED = 1.2
        self.BASE_WIDTH = 0.218
        self.PULSES_PER_METER = 5396.8
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
            self.rc_front.drive_forward_m2(speeds[1])
        else:
            self.rc_front.drive_backwards_m2(speeds[1])
        if speeds[2] >= 0:
            self.rc_rear.drive_forward_m1(speeds[2])
        else:
            self.rc_rear.drive_backwards_m1(speeds[2])
        if speeds[3] >= 0:
            self.rc_rear.drive_forward_m2(speeds[3])
        else:
            self.rc_rear.drive_backwards_m2(speeds[3])

    def nav_callback(self, twist):
        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED
        
        angular_z = twist.angular.z

        right_vel = linear_x + angular_z * self.BASE_WIDTH / 2.0
        left_vel = linear_x - angular_z * self.BASE_WIDTH / 2.0

        right_ticks = int(right_vel * self.PULSES_PER_METER)
        left_ticks = int(left_vel * self.PULSES_PER_METER)

        self.rc_front.drive_mixed_with_signed_speed(left_ticks, right_ticks)
        self.rc_rear.drive_mixed_with_signed_speed(left_ticks, right_ticks)


def main(args=None):
    rclpy.init(args=args)

    roboclaw_node = Roboclaw_Node()

    rclpy.spin(roboclaw_node)
 
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    roboclaw_node.serial_port.close()
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()