from rclpy.node import Node


class LoggerService:
    def __init__(self, node: Node):
        self.node = node

        self.logging_subscription = self.create_subscription(
            String,
            '/bno055/imu',
            self.logging_callback,
        10)

        # Assume IMU is initialized facing up
        self.z_orientation = True

    def logging_callback(self, msg):
        time = self.get_clock().now()
        self.get_logger().info('INFO @ %s' % time)

        # Task 1: Log velocity and orientation
        # TODO should these be a different type
        self.get_logger().info('Orientation (%s, %s, %s, %s)'
                               % (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                               throttle_duration_sec=1.0)
        self.get_logger().info('Velocity: (%s, %s, %s)'
                                % (msg.velocity.x, msg.velocity.y, msg.velocity.z),
                               throttle_duration_sec=1.0)

        # Task 2: Log when IMU flips over
        new_z_orientation = msg.linear_acceleration.z >= 0
        if self.z_orientation != new_z_orientation:
            self.get_logger().warn('IMU flipped over.')
            self.z_orientation = not self.z_orientation

        # Task 3: Log sudden acceleration changes
