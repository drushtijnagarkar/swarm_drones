#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import math
import time

class OffboardSquare(Node):
    def __init__(self):
        super().__init__('offboard_square')
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()
        self.phase = 0
        self.square_size = 5.0
        self.altitude = -5.0  # NED frame

        self.get_logger().info("OffboardSquare node started")

        # Arm + start offboard
        time.sleep(2)
        self.arm()
        self.enter_offboard_mode()

    def timer_callback(self):
        self.publish_offboard_mode()

        # Fly a square every 5 s per side
        t = time.time() - self.start_time
        phase_time = 5.0
        self.phase = int(t // phase_time) % 4

        x = y = 0.0
        if self.phase == 0:  x, y = self.square_size, 0.0
        elif self.phase == 1: x, y = self.square_size, self.square_size
        elif self.phase == 2: x, y = 0.0, self.square_size
        elif self.phase == 3: x, y = 0.0, 0.0

        self.publish_trajectory(x, y, self.altitude)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def arm(self):
        cmd = VehicleCommand()
        cmd.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def enter_offboard_mode(self):
        cmd = VehicleCommand()
        cmd.command = 176  # MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0   # Offboard mode
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardSquare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

