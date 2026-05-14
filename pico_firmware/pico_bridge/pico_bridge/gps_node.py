import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _ddmm_to_deg(ddmm: str, hemi: str) -> float:
    if not ddmm:
        return float('nan')
    val = float(ddmm)
    deg = int(val // 100)
    minutes = val - deg * 100
    decimal = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        decimal = -decimal
    return decimal


def _fix_quality_to_status(q: int) -> int:
    # NMEA GGA fix quality:
    # 0 invalid, 1 GPS fix, 2 DGPS, 3 PPS, 4 RTK fixed, 5 RTK float, 6 estimated
    if q == 0:
        return NavSatStatus.STATUS_NO_FIX
    if q == 2:
        return NavSatStatus.STATUS_SBAS_FIX
    if q in (4, 5):
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.sub = self.create_subscription(String, '/pico/gps_raw', self.on_line, 10)
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)

    def on_line(self, msg: String):
        line = msg.data.strip()
        if line.startswith('GPS:'):
            line = line[4:].strip()

        if '*' in line:
            line = line.split('*', 1)[0]

        if not (line.startswith('$GNGGA') or line.startswith('$GPGGA')):
            return

        fields = line.split(',')
        if len(fields) < 10:
            return

        try:
            lat = _ddmm_to_deg(fields[2], fields[3])
            lon = _ddmm_to_deg(fields[4], fields[5])
            quality = int(fields[6]) if fields[6] else 0
            altitude = float(fields[9]) if fields[9] else float('nan')
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'failed to parse GGA: {line!r} ({e})')
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'

        fix.status.status = _fix_quality_to_status(quality)
        fix.status.service = (NavSatStatus.SERVICE_GPS
                              | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_GALILEO
                              | NavSatStatus.SERVICE_COMPASS)

        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = altitude

        fix.position_covariance = [0.0] * 9
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
