import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


class SerialParserNode(Node):
    def __init__(self):
        super().__init__('serial_parser')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.imu_pub = self.create_publisher(String, '/pico/imu_raw', 50)
        self.gps_pub = self.create_publisher(String, '/pico/gps_raw', 10)
        self.mag_pub = self.create_publisher(String, '/pico/mag_raw', 50)

        self.get_logger().info(f'Opening {port} @ {baud} baud')
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.timer = self.create_timer(0.001, self.poll_serial)

    def poll_serial(self):
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline()
                if not raw:
                    return
                try:
                    line = raw.decode('ascii', errors='replace').strip()
                except Exception:
                    return
                if not line:
                    return

                msg = String()
                msg.data = line
                if line.startswith('IMU:'):
                    self.imu_pub.publish(msg)
                elif line.startswith('GPS:'):
                    self.gps_pub.publish(msg)
                elif line.startswith('MAG:'):
                    self.mag_pub.publish(msg)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
