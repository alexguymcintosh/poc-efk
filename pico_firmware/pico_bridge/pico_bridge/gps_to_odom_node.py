import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry


# Equirectangular projection on the WGS84 equatorial radius. Accurate to a few
# cm over hundreds of meters at mid-latitudes — fine for a local odom frame.
WGS84_A = 6378137.0


class GpsToOdomNode(Node):
    def __init__(self):
        super().__init__('gps_to_odom')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.datum_lat = None  # radians
        self.datum_lon = None
        self.datum_alt = None

        self.sub = self.create_subscription(NavSatFix, '/fix', self.on_fix, 10)
        self.pub = self.create_publisher(Odometry, '/odometry/gps', 10)

    def on_fix(self, msg: NavSatFix):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        alt = msg.altitude if not math.isnan(msg.altitude) else 0.0

        if self.datum_lat is None:
            self.datum_lat = lat
            self.datum_lon = lon
            self.datum_alt = alt
            self.get_logger().info(
                f'Datum set: lat={msg.latitude:.7f} lon={msg.longitude:.7f} alt={alt:.2f}')

        x = WGS84_A * math.cos(self.datum_lat) * (lon - self.datum_lon)  # east
        y = WGS84_A * (lat - self.datum_lat)                              # north
        z = alt - self.datum_alt

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id

        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        out.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            # NavSatFix carries a 3x3 ENU position covariance — copy into the
            # top-left of the 6x6 pose covariance.
            for i in range(3):
                for j in range(3):
                    cov[i * 6 + j] = msg.position_covariance[i * 3 + j]
        else:
            cov[0]  = 25.0   # x variance (m^2), ~5 m std
            cov[7]  = 25.0   # y
            cov[14] = 100.0  # z (altitude noisier)
        # Orientation not measured by GPS — signal "unknown" so the EKF ignores
        # these rows even if odom0_config later changes.
        cov[21] = -1.0
        cov[28] = -1.0
        cov[35] = -1.0
        out.pose.covariance = cov

        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = -1.0
        out.twist.covariance = twist_cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
