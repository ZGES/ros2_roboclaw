import rclpy
from rclpy.node import Node

import serial
import math

from src.roboclaw_driver import SerialPort, Roboclaw

from roboclaw_comm.msg import Speed
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations

class Odom():
    def __init__(self, node):
        self.node = node
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_theta = 0.0
        #self.last_enc_left_front = 0.0
        #self.last_enc_right_front = 0.0
        self.last_enc_left = 0.0
        self.last_enc_right = 0.0
        self.last_enc_time = self.node.get_clock().now()

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle 

    def update_odom(self, enc_l, enc_r):
        if enc_l == -1 or enc_r == -1:
            self.node.get_logger().error('ENCODERS ERROR!')
        
        #front_left_ticks = enc_fl - self.last_enc_left_front
        #front_right_ticks = enc_fr - self.last_enc_right_front
        left_ticks = -enc_l + self.last_enc_left
        right_ticks = -enc_r + self.last_enc_right
        
        #self.last_enc_left_front = enc_fl
        #self.last_enc_right_front = enc_fr
        self.last_enc_left = enc_l
        self.last_enc_right = enc_r

        #left_ticks = (front_left_ticks + rear_left_ticks)/2
        #right_ticks = (front_right_ticks + rear_right_ticks)/2

        dist_left = left_ticks / self.node.TICKS_PER_METER
        dist_right = right_ticks / self.node.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.node.get_clock().now()
        d_time = (current_time.nanoseconds - self.last_enc_time.nanoseconds) // 1e9 + ((current_time.nanoseconds - self.last_enc_time.nanoseconds) % 1e9 * 1e-9)
        self.last_enc_time = current_time

        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * math.cos(self.cur_theta)
            self.cur_y += dist * math.sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.node.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (math.sin(d_theta + self.cur_theta) - math.sin(self.cur_theta))
            self.cur_y -= r * (math.cos(d_theta + self.cur_theta) - math.cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta


class Roboclaw_Node(Node):

    def __init__(self):
        super().__init__('roboclaw_node')
        self.MIN_TICKS = 1500
        self.MAX_TICKS = 4500
        self.BASE_WIDTH = 0.224
        self.TICKS_PER_METER = 5500
        #self.FRONT_RC_ADDRESS = 129
        self.REAR_RC_ADDRESS = 129
        self.SERIAL_PORT = "/dev/serial1"
        self.BAUD_RATE = 38400
        self.TIMEOUT = 0.7
        self.serial_port = SerialPort(serial.Serial(port=self.SERIAL_PORT, baudrate=self.BAUD_RATE, timeout=self.TIMEOUT))
        #self.rc_front = Roboclaw(self.serial_port, self.FRONT_RC_ADDRESS)
        self.rc_rear = Roboclaw(self.serial_port, self.REAR_RC_ADDRESS)
        self.odom = Odom(self)

        #self.rc_front.reset_quad_encoder_counters()
        self.rc_rear.reset_quad_encoder_counters()

        self.manual_sub = self.create_subscription(
            Speed,
            'manual',
            self.manual_callback,
            50)
        self.manual_sub # prevent unused variable warning

        self.nav_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.nav_callback,
            50)
        self.nav_sub

        self.odom_pub = self.create_publisher(Odometry, 'capo/odom', 50)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.odom_callback)

    def manual_callback(self, msg):
        speeds = msg.speed
        self.get_logger().info('Received message: front left: {}, front right: {}, rear left: {}, rear right: {}'.format(speeds[0],speeds[1],speeds[2], speeds[3]))
        #if speeds[0] >= 0:
        #    self.rc_front.drive_forward_m2(speeds[0])
        #else:
        #    self.rc_front.drive_backwards_m2(abs(speeds[0]))
        #if speeds[1] >= 0:
        #    self.rc_front.drive_forward_m1(speeds[1])
        #else:
        #    self.rc_front.drive_backwards_m1(abs(speeds[1]))
        if speeds[2] >= 0:
            self.rc_rear.drive_forward_m1(speeds[2])
        else:
            self.rc_rear.drive_backwards_m1(abs(speeds[2]))
        if speeds[3] >= 0:
            self.rc_rear.drive_forward_m2(speeds[3])
        else:
            self.rc_rear.drive_backwards_m2(abs(speeds[3]))

    def nav_callback(self, twist):
        self.get_logger().info('Received message: linear x = {}, angular z = {}'.format(twist.linear.x, twist.angular.z))
        
        linear_x = twist.linear.x
        
        angular_z = twist.angular.z

        angle_component = 1.2 * angular_z * self.BASE_WIDTH / 2.0
        right_vel = linear_x + angle_component
        left_vel = linear_x - angle_component
        
        if right_vel > 0.00001:
            right_ticks = min(self.MIN_TICKS + abs(right_vel * self.TICKS_PER_METER), self.MAX_TICKS)
        elif right_vel < -0.00001:
            right_ticks = max(-self.MIN_TICKS - abs(right_vel * self.TICKS_PER_METER), -self.MAX_TICKS)
        else:
            right_ticks = 0

        if left_vel > 0.00001:
            left_ticks = min(self.MIN_TICKS + abs(left_vel * self.TICKS_PER_METER), self.MAX_TICKS)
        elif left_vel < -0.00001:
            left_ticks = max(-self.MIN_TICKS - abs(left_vel * self.TICKS_PER_METER), -self.MAX_TICKS)
        else:
            left_ticks = 0
        
        self.get_logger().info('Set wheels ticks: LEFT - {}, RIGHT - {}'.format(left_ticks, right_ticks))
        #self.rc_front.drive_mixed_with_signed_speed(int(right_ticks), int(left_ticks))
        self.rc_rear.drive_mixed_with_signed_speed(int(-left_ticks), int(-right_ticks))

    def odom_callback(self):
        #enc_fl, _ = self.rc_front.read_quad_encoder_register_m2()
        #enc_fr, _ = self.rc_front.read_quad_encoder_register_m1()
        enc_l, _ = self.rc_rear.read_quad_encoder_register_m1()
        enc_r, _ = self.rc_rear.read_quad_encoder_register_m2()

        self.get_logger().info('Encoders reads: left = {}, right = {}'.format(enc_l, enc_r))

        vel_x, vel_theta = self.odom.update_odom(enc_l, enc_r)

        quat = tf_transformations.quaternion_from_euler(0, 0, -self.odom.cur_theta)

        odometry = Odometry()
        quaternion = Quaternion()

        quaternion.x = quat[0]
        quaternion.y = quat[1]
        quaternion.z = quat[2]
        quaternion.w = quat[3]

        odometry.header.stamp = self.odom.last_enc_time.to_msg()
        odometry.header.frame_id = 'odom'

        odometry.child_frame_id = 'base_link'

        odometry.pose.pose.position.x = self.odom.cur_x
        odometry.pose.pose.position.y = self.odom.cur_y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation = quaternion

        odometry.pose.covariance[0] = 1e-2
        odometry.pose.covariance[7] = 1e-2
        odometry.pose.covariance[14] = 1e5
        odometry.pose.covariance[21] = 1e5
        odometry.pose.covariance[28] = 1e5
        odometry.pose.covariance[35] = 1e-2

        odometry.twist.twist.linear.x = vel_x
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.linear.z = 0.0
        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0
        odometry.twist.twist.angular.z = vel_theta
        odometry.twist.covariance = odometry.pose.covariance

        self.odom_pub.publish(odometry)
        self.get_logger().info('Odometry published: pose_x = {}  pose_y = {}, linear_x = {}, angular_z = {}'.format(self.odom.cur_x, self.odom.cur_y, vel_x, vel_theta))

    


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