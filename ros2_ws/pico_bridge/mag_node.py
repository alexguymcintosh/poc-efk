import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu


class MagNode(Node):
    def __init__(self):
        super().__init__('mag_node')

        # Magnetic declination (radians). Positive = East.
        # Set for your location: Melbourne ≈ +0.220 rad (+12.6°).
        # Tune empirically: if EKF heading reads ~12° low, add 0.2 rad.
        self.declare_parameter('declination', 0.0)
        self.declination = self.get_parameter('declination').get_parameter_value().double_value

        self.sub = self.create_subscription(String, '/pico/mag_raw', self.on_line, 50)
        self.pub = self.create_publisher(Imu, '/imu/data', 50)

        self.get_logger().info(f'mag_node ready. declination={math.degrees(self.declination):.1f}°')

    def on_line(self, msg: String):
        line = msg.data.strip()
        if not line.startswith('MAG:'):
            return
        parts = line[4:].split(',')
        if len(parts) != 3:
            self.get_logger().warn(f'unexpected MAG field count: {len(parts)}')
            return
        try:
            mx, my, mz = (float(p) for p in parts)
        except ValueError:
            self.get_logger().warn(f'failed to parse MAG line: {line!r}')
            return

        # Chip orientation: x forward, y left, z up (ENU-aligned on rover body).
        # yaw_enu = atan2(mx, my) gives angle from East, CCW positive.
        # Derivation: when facing East (yaw=0), mx≈0 and my=B_h (horiz field, y=left=North).
        # atan2(0, B_h) = 0 ✓. When facing North (yaw=π/2), mx=B_h, my≈0 → atan2(B_h,0)=π/2 ✓.
        yaw = math.atan2(mx, my) + self.declination
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # normalise to [-π, π]

        # Quaternion for pure yaw rotation (rotation about z-axis)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'imu_link'

        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = qz
        out.orientation.w = qw
        # EKF fuses only yaw ([8]). Roll/pitch set large — we have no estimate.
        out.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 0.1,   # yaw variance ~18° 1-sigma
        ]

        # Angular velocity and linear acceleration not provided by this node.
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
