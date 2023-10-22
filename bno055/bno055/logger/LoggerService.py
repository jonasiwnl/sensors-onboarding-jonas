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

        # Assume IMU is initialized facing up
        self.z_orientation = True

        # Assume IMU is initialized not moving
        self.last_acceleration = None
        self.last_time = node.get_clock().now()

    def logging_callback(self, msg):
        time = self.node.get_clock().now()
        # self.node.get_logger().info('INFO @ %s' % time)

        # Task 1: Log velocity and orientation
        self.node.get_logger().info('Orientation (%.2f, %.2f, %.2f, %.2f)'
                               % (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                               throttle_duration_sec=1.0)
        self.node.get_logger().info('Velocity: (%.2f, %.2f, %.2f)'
                                % (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                               throttle_duration_sec=1.0)

        # Task 2: Log when IMU flips over
        # If the gravity vector is facing down, z orientation is up
        new_z_orientation = msg.linear_acceleration.z <= 0
        if self.z_orientation != new_z_orientation:
            self.node.get_logger().warn('IMU flipped over.')
            self.z_orientation = not self.z_orientation

        """
        # Task 3: Log sudden acceleration changes
        if self.last_acceleration:
            distance = sqrt(
                pow(msg.linear_acceleration.x - self.last_acceleration.x, 2) +
                pow(msg.linear_acceleration.y - self.last_acceleration.y, 2) +
                pow(msg.linear_acceleration.z - self.last_acceleration.z, 2) )

            # TODO ?
            if distance / (time - self.last_time) > self.ACCELERATION_THRESHOLD:
                self.node.get_logger().warn('Sudden acceleration change.')
        self.last_acceleration = msg.linear_acceleration
        self.last_time = time
        """
