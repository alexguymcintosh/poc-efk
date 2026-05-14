import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu


DEG_TO_RAD = math.pi / 180.0


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.sub = self.create_subscription(String, '/pico/imu_raw', self.on_line, 50)
        self.pub = self.create_publisher(Imu, '/imu/data_raw', 50)

    def on_line(self, msg: String):
        line = msg.data.strip()
        if not line.startswith('IMU:'):
            return
        payload = line[4:]
        parts = payload.split(',')
        if len(parts) != 7:
            self.get_logger().warn(f'unexpected IMU field count: {len(parts)}')
            return

        try:
            ax, ay, az, gx, gy, gz, _temp = (float(p) for p in parts)
        except ValueError:
            self.get_logger().warn(f'failed to parse IMU line: {line!r}')
            return

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu_link'

        # Orientation not measured by raw MPU-6050 — signal "no estimate".
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = 0.0
        out.orientation.w = 1.0
        out.orientation_covariance = [-1.0] + [0.0] * 8

        out.angular_velocity.x = gx * DEG_TO_RAD
        out.angular_velocity.y = gy * DEG_TO_RAD
        out.angular_velocity.z = gz * DEG_TO_RAD
        out.angular_velocity_covariance = [-1.0] + [0.0] * 8

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = [-1.0] + [0.0] * 8

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
