#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import *

class DynamixelJointPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.read_and_publish)

        self.portHandler = PortHandler('/dev/ttyUSB0')
        self.packetHandler = PacketHandler(1.0)  # Protocol 1.0 for AX-12A and RX-64

        self.motor_ids = [9, 10]
        self.motor_models = {}  # ID -> model number
        self.position_addresses = {}  # ID -> address
        self.position_scales = {}     # ID -> scale factor to radians

        if not self.portHandler.openPort():
            self.get_logger().error('Failed to open port')
        if not self.portHandler.setBaudRate(1000000):
            self.get_logger().error('Failed to set baud rate')

        self.detect_motors()

    def detect_motors(self):
        for motor_id in self.motor_ids:
            # Force override for known RX-64s
            if motor_id in [9, 10]:
                model_number = 320  # RX-64
                self.get_logger().info(f"ID {motor_id}: Forced RX-64 override")
            else:
                model_number, result, error = self.packetHandler.ping(self.portHandler, motor_id)
                if result != COMM_SUCCESS:
                    self.get_logger().warn(f"ID {motor_id}: Ping failed")
                    continue

            self.motor_models[motor_id] = model_number

            if model_number == 64:  # AX-12A
                self.position_addresses[motor_id] = 36
                self.position_scales[motor_id] = (300.0 / 1023.0) * (3.14159 / 180.0)
                self.get_logger().info(f"ID {motor_id}: AX-12A detected")
            elif model_number == 320:  # RX-64
                self.position_addresses[motor_id] = 36
                self.position_scales[motor_id] = (300.0 / 4095.0) * (3.14159 / 180.0)
                self.get_logger().info(f"ID {motor_id}: RX-64 detected")
            else:
                self.get_logger().warn(f"ID {motor_id}: Unknown model {model_number}, using AX-12A defaults")
                self.position_addresses[motor_id] = 36
                self.position_scales[motor_id] = (300.0 / 1023.0) * (3.14159 / 180.0)

def read_and_publish(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = []
    msg.position = []

    for motor_id in self.motor_ids:
        if motor_id not in self.position_addresses:
            continue

        addr = self.position_addresses[motor_id]
        scale = self.position_scales[motor_id]

        dxl_pos, result, error = self.packetHandler.read2ByteTxRx(
            self.portHandler, motor_id, addr
        )

        if result != COMM_SUCCESS:
            self.get_logger().warn(f'Comm error on ID {motor_id}')
            continue

        radians = dxl_pos * scale

        # Deadband filter: only publish if change > threshold
        last_pos = getattr(self, f'last_pos_{motor_id}', None)
        if last_pos is None or abs(radians - last_pos) > 0.005:  # ~0.3°
            setattr(self, f'last_pos_{motor_id}', radians)
            msg.name.append(f'joint_{motor_id}')
            msg.position.append(radians)

    if msg.name:
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
