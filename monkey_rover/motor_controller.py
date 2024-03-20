"""
Rover motor differential drive controller and motor hardware interface.

Written by:
    1. Kean Hao
    2. Jia Hao
Last Updated: 8 May 2023
"""

from math import pi, asin, cos, sin
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ament_index_python.packages import get_package_share_directory

import os
import time
import copy
import rclpy
import canopen

#Constants
# NODE_ID = 0x0E #Arm's subcontroller node id

# WHEEL_BASE = 1 # Distance between center part of left and right wheels
# WHEEL_DIAMETER = 0.2372
# GEAR_RATIO = 40
# WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER
# MAX_RPM = 3000

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

        # Get robot parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.wheel_circumference = pi * self.wheel_diameter
        self.max_rpm = self.get_parameter('max_rpm').value

        # Initilize odometry value
        self.time_prev = 0
        self.left_rpm_prev = 0
        self.right_rpm_prev = 0

        #Set parameters callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        #Set publisher and subscriber
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel_received, 5)
        self.get_logger().info(f"{self.get_name()} started")

    def on_cmd_vel_received(self, rosmsg: Twist):
        """Call when cmd_vel received."""
        # Calculate left and right wheel velocity
        rot_vel = (rosmsg.angular.z * self.wheel_base) / 2.0 # Wheel velocity based on rotation
        left_vel = rosmsg.linear.x - rot_vel
        right_vel = rosmsg.linear.x + rot_vel

        # Calculate wheel rpm
        left_rpm = int(left_vel / self.wheel_circumference * 60 * self.gear_ratio)
        right_rpm = int(right_vel / self.wheel_circumference * 60 * self.gear_ratio)
        self.get_logger().info(f"left_rpm: {left_rpm}, right_rpm: {right_rpm}")

        # Normalize RPM values if exceed maximum RPM
        highest_rpm = max(abs(left_rpm), abs(right_rpm))
        if highest_rpm > self.max_rpm:
            left_rpm = left_rpm / highest_rpm * self.max_rpm
            right_rpm = right_rpm / highest_rpm * self.max_rpm
            self.get_logger().info(f"left_rpm (normalized): {left_rpm}, right_rpm (normalized): {right_rpm}")

        # Set target velocity to motor driver
        target_velocity_left = motor_node.sdo[0x60FF] # Target_velocity L
        target_velocity_right = motor_node.sdo[0x68FF] # Target_velocity R
        try:
            target_velocity_left.raw = left_rpm
            target_velocity_right.raw = right_rpm
        except canopen.sdo.SdoCommunicationError:
            self.get_logger().error("Failed to set target speed on motor driver.")

        # TODO: Read actual motor velocity
        # velocity_actual_value = motor_node.sdo[0x606C]
        # self.get_logger().info(f'Actual velocity: {velocity_actual_value.raw} RPM')

    def tpdo1_callback(self, msg):
        """Call when Transmit PDO received."""
        for var in msg:
            self.get_logger().info(f'{var.name} = {var.raw}')

    def parameters_callback(self, params):
        """Call when robot parameter changed."""
        #Update values
        for param in params:
            if param.name == "max_rpm":
                self.max_rpm = param.value
                self.get_logger().info(f'Set maximum RPM to {self.max_rpm}')

        return SetParametersResult(successful=True)

        # #Encode and send message
        # msg = frame.encode_message()
        # self.bus.send(msg)
        # self.get_logger().info(f"TRANSMIT\tid: {hex(msg.arbitration_id)}\tdata: {hex(msg.data[0])}|{hex(msg.data[1])}|{hex(msg.data[2])}|{hex(msg.data[3])}|{hex(msg.data[4])}|{hex(msg.data[5])}|{hex(msg.data[6])}|{hex(msg.data[7])}")

def main(args=None):
    """Run when this script is called"""
    rclpy.init(args=args)

    #Initialize CANopen network
    network = canopen.Network()
    #network.connect(bustype='socketcan', channel='can0', bitrate=500000)
    network.connect(interface='seeedstudio', channel='/dev/ttyUSB0', baudrate=2000000, bitrate=500000)

    global motor_node
    motor_node = network.add_node(0x33,
        os.path.join(get_package_share_directory('monkey_rover'), 'eds', 'LDS Driver.eds'))

    # Set NMT state
    # STOPPED, PRE-OPERATIONAL, OPERATIONAL
    motor_node.nmt.state = 'PRE-OPERATIONAL'

    # Set operation mode to speed control mode
    try:
        print("Setting Opreation Mode to Speed Control Mode")
        operation_mode = motor_node.sdo[0x6060] # Moes_of_operation
        operation_mode.raw = 0x03 # Speed control mode
    except canopen.sdo.SdoCommunicationError:
        # TODO: Control mode not set
        print("Failed to set Operation Mode to Speed Control Mode")

    # Set control word for left and right motor
    try:
        print("Setting control word")
        cw_left = motor_node.sdo[0x6040]
        cw_left.raw = 6
        cw_left.raw = 7
        cw_left.raw = 15

        cw_right = motor_node.sdo[0x6840]
        cw_right.raw = 6
        cw_right.raw = 7
        cw_right.raw = 15
    except canopen.sdo.SdoCommunicationError:
        print("Failed to set control word for left and right motor")

    """
    # Setup TPDO receiver
    motor_node.tpdo.read()
    motor_node.tpdo[1].clear()
    motor_node.tpdo[1].add_variable(0x606C, 0x00) #Actual motor velocity
    motor_node.tpdo[1].trans_tyoe = 0xFE #Asynchronous: Triggered by an internal event
    motor_node.tpdo[1].event_timer = 100 #Timer in ms
    motor_node.tpdo[1].enabled = True
    motor_node.tpdo.save()
    """

    motor_controller = MotorController()
    # motor_node.tpdo[1].add_callback(motor_controller.tpdo1_callback) # Add callback

    # Start PDO
    motor_node.nmt.state = 'OPERATIONAL'

    # Start motor controller node
    motor_controller.time_prev = time.time() #Update time
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()
    network.disconnect()

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
    
