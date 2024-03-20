"""
Python script for testing LDS90B96G-2 motor driver control using CANopen
communication protocol.

Written by:
    1. Kean Hao
    2. Jia Hao
Last Updated: 8 May 2023
"""

import canopen

if __name__ == "__main__":
    #Initialize CANopen network
    network = canopen.Network()
    network.connect(bustype='socketcan', channel='can0', bitrate=500000)

    global motor_node
    motor_node = network.add_node(0x33, "/home/ubuntu/ros2_nav/src/monkey_rover/eds/LDS Driver.eds")

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

    # Set left and right motor velocity
    target_velocity_left = motor_node.sdo[0x60FF] # Target_velocity L
    target_velocity_left.raw = 0
    target_velocity_right = motor_node.sdo[0x68FF] # Target_velocity R
    target_velocity_right.raw = 0

    # velocity_value_left = motor_node.sdo[0x606C] #Velocity Actual Value L
    # print(velocity_value_left.read())
    