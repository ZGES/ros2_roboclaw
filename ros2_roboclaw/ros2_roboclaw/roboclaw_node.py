import rclpy
from rclpy.node import Node

from roboclaw_comm.msg import Speed

import logging
import logging.config
import sys
import threading
import traceback
import serial
import os
import time
import math
import struct
import sys

class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0
        
    def close(self):
        self.__port.close()
    
    def reset_checksum(self, value=0x0):
        self.__checksum = value
        
    def get_checksum(self, mask=0x7F):
        return self.__checksum & mask
   
    def send_command(self, address, command):
        self.write_byte(address)
        self.write_byte(command)
        
    def write_byte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>B', val))
        
class Roboclaw(object):
    def __init__(self, port, rc_address=128):
        self.__port = port
        self.__rc_address = rc_address
        
    def drive_forward_m1(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 0)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())
        
    def drive_backwards_m1(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 1)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_forward_m2(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 4)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())
          
    def drive_backwards_m2(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 5)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())
        
def test():
    print('Hello\n')
    FRONT_RC_ADDRESS = 129
    REAR_RC_ADDRESS = 128


    SERIAL_PORT = "/dev/serial1"
    BAUD_RATE = 38400
    TIMEOUT = 0.7
    _serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
    _serial_port = SerialPort(_serial)

    roboclaw_front = Roboclaw(_serial_port, FRONT_RC_ADDRESS)
    print("Front roboclaw\n")

    roboclaw_rear = Roboclaw(_serial_port, REAR_RC_ADDRESS)
    print("Rare roboclaw\n")

    roboclaw_rear.drive_forward_m1(0);
    roboclaw_rear.drive_forward_m2(0);

    roboclaw_front.drive_forward_m1(0);
    roboclaw_front.drive_forward_m2(0);

    time.sleep(10)

    roboclaw_rear.drive_forward_m1(20);
    roboclaw_rear.drive_forward_m2(20);

    roboclaw_front.drive_forward_m1(20);
    roboclaw_front.drive_forward_m2(20);

    time.sleep(10)

    roboclaw_rear.drive_backwards_m1(20);
    roboclaw_rear.drive_backwards_m1(50);

    roboclaw_front.drive_backwards_m1(20);
    roboclaw_front.drive_backwards_m1(50);

    time.sleep(10)

    roboclaw_rear.drive_forward_m1(0);
    roboclaw_rear.drive_forward_m2(0);

    roboclaw_front.drive_forward_m1(0);
    roboclaw_front.drive_forward_m2(0);

    _serial.close()

    print('serial.close()\n')


class Roboclaw_Node(Node):

    def __init__(self):
        super().__init__('roboclaw_node')
        self.subscription = self.create_subscription(
            Speed,
            'roboclaw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
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