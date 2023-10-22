from math import sqrt

from rclpy.node import Node
from sensor_msgs.msg import Imu


class LoggerService:
    ACCELERATION_THRESHOLD = 10.0

    def __init__(self, node: Node):
        self.node = node

        self.logging_subscription = self.node.create_subscription(
            Imu,
            '/bno055/imu',
            self.logging_callback,
        10)

        # Assume IMU is initialized not moving
        self.velocity.x = self.velocity.y = self.velocity.z = 0
        self.acceleration.x = self.acceleration.y = self.acceleration.z = 0
        self.time = node.get_clock().now().nanoseconds * 1e-9
        self.z_orientation = False # Assume IMU is initialized facing up

    def logging_callback(self, msg):
        # Task 1: Log linear velocity and orientation
        new_time = self.node.get_clock().now().nanoseconds * 1e-9
        delta = self.time - new_time
        self.velocity.x += msg.linear_acceleration.x * delta
        self.velocity.y += msg.linear_acceleration.y * delta
        self.velocity.z += msg.linear_acceleration.z * delta

        self.node.get_logger().info('Orientation (%.2f, %.2f, %.2f, %.2f)'
                               % (msg.orientation.x, msg.orientation.y,
                                  msg.orientation.z, msg.orientation.w),
                               throttle_duration_sec=1.0)
        self.node.get_logger().info('Velocity: (%.2f, %.2f, %.2f)'
                                % (self.velocity.x, self.velocity.y, self.velocity.z),
                               throttle_duration_sec=1.0)

        # Task 2: Log when IMU flips over
        # If the gravity vector is facing down, z orientation is up
        new_z_orientation = msg.linear_acceleration.z <= 0
        if self.z_orientation != new_z_orientation:
            self.node.get_logger().warn('IMU flipped over.')
            self.z_orientation = not self.z_orientation

        # Task 3: Log sudden acceleration changes
        accel_delta = sqrt(
            pow(msg.linear_acceleration.x - self.acceleration.x, 2) +
            pow(msg.linear_acceleration.y - self.acceleration.y, 2) +
            pow(msg.linear_acceleration.z - self.acceleration.z, 2) )

        # TODO ?
        if accel_delta / delta > self.ACCELERATION_THRESHOLD:
            self.node.get_logger().warn('Sudden acceleration change.')

        self.acceleration = msg.linear_acceleration
        self.time = new_time
