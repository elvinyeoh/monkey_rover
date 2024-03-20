from math import pi, asin, cos, sin
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

import os
import time
import copy
import rclpy

class MotorController(Node):
    """Differential drive controller"""
    def __init__(self):
        super().__init__('motor_controller')
        self.declare_parameter('wheel_base', 1.0,
            ParameterDescriptor(description='Distance between center part of left and right wheels'))
        self.declare_parameter('wheel_diameter', 0.2372,
            ParameterDescriptor(description='Diameter of driving wheels'))
        self.declare_parameter('gear_ratio', 40,
            ParameterDescriptor(description='Motor RPM to Wheel RPM ratio'))
        self.declare_parameter('max_rpm', 3000,
            ParameterDescriptor(description='Maximum motor RPM'))

        # self.declare_parameters(
        #     namespace='',
        #     parameters=[
        #         ('wheel_base', 1.0),
        #         ('wheel_diameter', 0.2372),
        #         ('gear_ratio', 40),
        #         ('max_rpm', 3000)
        # ])

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.wheel_circumference = pi * self.wheel_diameter
        self.max_rpm = self.get_parameter('max_rpm').value

        #Initialize value
        self.time_prev = 0
        self.left_rpm_prev = 0
        self.right_rpm_prev = 0

        self.odom_prev = Odometry()
        #TODO: Read robot current pose
        self.odom_prev.header.frame_id = 'odom'
        self.odom_prev.child_frame_id = 'base_footprint'
        self.odom_prev.pose.pose.position.x = 0.0
        self.odom_prev.pose.pose.position.y = 0.0
        self.odom_prev.pose.pose.orientation.z = 0.0
        for i in range(0, 36):
            self.odom_prev.pose.covariance[i] = 0
        self.odom_prev.pose.covariance[0] = 0.01
        self.odom_prev.pose.covariance[7] = 0.01
        self.odom_prev.pose.covariance[14] = 0.01
        self.odom_prev.pose.covariance[21] = 0.1
        self.odom_prev.pose.covariance[28] = 0.1
        self.odom_prev.pose.covariance[35] = 0.1

        #Set parameters callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        #Set publisher and subscriber
        self.odom_publisher = self.create_publisher(Odometry, '/wheel/odometry', 5)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel_received, 5)
        self.get_logger().info(f"{self.get_name()} started")

    def on_cmd_vel_received(self, rosmsg: Twist):
        """Call when cmd_vel received."""
        #Calculate left and right wheel velocity
        rot_vel = (rosmsg.angular.z * self.wheel_base) / 2.0 # Wheel velocity based on rotation
        left_vel = rosmsg.linear.x - rot_vel
        right_vel = rosmsg.linear.x + rot_vel

        #Calculate wheel rpm
        left_rpm = int(left_vel / self.wheel_circumference * 60 * self.gear_ratio)
        right_rpm = int(right_vel / self.wheel_circumference * 60 * self.gear_ratio)
        self.get_logger().info(f"left_rpm: {left_rpm}, right_rpm: {right_rpm}")

        #Normalize RPM values if exceed maximum RPM
        highest_rpm = max(abs(left_rpm), abs(right_rpm))
        if highest_rpm > self.max_rpm:
            left_rpm = left_rpm / highest_rpm * self.max_rpm
            right_rpm = right_rpm / highest_rpm * self.max_rpm
            self.get_logger().info(f"left_rpm (normalized): {left_rpm}, right_rpm (normalized): {right_rpm}")

        self.left_rpm = left_rpm
        self.right_rpm = right_rpm

        self.test_odom()

    def test_odom(self):
        # Get time elapsed from previous RPM reading
        time_curr = time.time()
        time_diff = time_curr - self.time_prev
        self.get_logger().info(f'time_diff: {time_diff}s')

        # Calculate average RPM
        # TODO: Read RPM values from motor driver
        left_rpm = self.left_rpm
        right_rpm = self.right_rpm
        left_rpm_avg = (left_rpm + self.left_rpm_prev) / 2
        right_rpm_avg = (right_rpm + self.right_rpm_prev) / 2
        self.get_logger().info(f'left_rpm_avg: {left_rpm_avg}RPM')
        self.get_logger().info(f'right_rpm_avg: {right_rpm_avg}RPM')

        # Calculate velocity
        left_vel = left_rpm_avg * self.wheel_circumference / (60 * self.gear_ratio)
        right_vel = right_rpm_avg * self.wheel_circumference / (60 * self.gear_ratio)
        self.get_logger().info(f'left_vel: {left_vel}m/s')
        self.get_logger().info(f'right_vel: {right_vel}m/s')

        # Calculate distance travelled
        left_dist = left_vel * time_diff
        right_dist = right_vel * time_diff
        self.get_logger().info(f'left_dist: {left_dist}m')
        self.get_logger().info(f'right_dist: {right_dist}m')

        cycle_dist = (left_dist + right_dist) / 2 # Average distance travelled
        cycle_angle = asin((right_dist - left_dist)/self.wheel_base) # Angle turned
        avg_angle = cycle_angle/2 + self.odom_prev.pose.pose.orientation.z # Average angle during last cycle
        # Normalize angle
        if avg_angle > pi:
            avg_angle = avg_angle - 2*pi
        elif avg_angle < -pi:
            avg_angle = avg_angle + 2*pi

        # Calculate new pose
        odom = copy.deepcopy(self.odom_prev)
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.odom_prev.pose.pose.position.x + cos(avg_angle)*cycle_dist
        odom.pose.pose.position.y = self.odom_prev.pose.pose.position.y + sin(avg_angle)*cycle_dist
        odom.pose.pose.orientation.z = cycle_angle + self.odom_prev.pose.pose.orientation.z
        # Normalize angle
        if odom.pose.pose.orientation.z > pi:
            odom.pose.pose.orientation.z = odom.pose.pose.orientation.z - 2*pi
        elif odom.pose.pose.orientation.z < -pi:
            odom.pose.pose.orientation.z = odom.pose.pose.orientation.z + 2*pi

        # Calculate linear and angular velocity
        odom.twist.twist.linear.x = (left_vel + right_vel) / 2
        odom.twist.twist.angular.z = (right_vel - left_vel) / self.wheel_base

        # Publish odometry in quarternion format
        odom_quat = copy.deepcopy(odom)
        quat = quaternion_from_euler(0, 0, odom.pose.pose.orientation.z)
        odom_quat.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.odom_publisher.publish(odom_quat)

        # Update parameters
        self.odom_prev = odom
        self.time_prev = time_curr
        self.left_rpm_prev = left_rpm
        self.right_rpm_prev = right_rpm

    def parameters_callback(self, params):
        """Call when parameter changed."""
        #Update values
        for param in params:
            if param.name == "max_rpm":
                self.max_rpm = param.value
                self.get_logger().info(f'Set maximum RPM to {self.max_rpm}')

        return SetParametersResult(successful=True)

def main(args=None):
    """Run when this script is called"""
    rclpy.init(args=args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

if __name__ == '__main__':
    main()