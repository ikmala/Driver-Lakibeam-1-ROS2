import rclpy
from rclpy.node import Node
import socket
import struct
import math
from sensor_msgs.msg import LaserScan


class LakiBeam1Driver(Node):
    def __init__(self):
        super().__init__('lakibeam1_driver')

        self.declare_parameter('port', 2368)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('inverted', False)
        self.declare_parameter('resolution_deg', 1.0)
        self.declare_parameter('range_min', 0.01)
        self.declare_parameter('range_max', 100.0)
        self.declare_parameter('debug', False)

        port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        self.inverted = bool(self.get_parameter('inverted').value)
        self.resolution_deg = float(self.get_parameter('resolution_deg').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.num_steps = int(360.0 / self.resolution_deg)
        self.frame = [float('inf')] * self.num_steps
        self.last_angle_deg = None
        self.last_publish_time = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.1)
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(
            f"LakiBeam-1 driver started on UDP port {port} | "
            f"resolution={self.resolution_deg} deg, steps={self.num_steps}"
        )
        self.timer = self.create_timer(0.001, self.read_packet)

    def read_packet(self):
        try:
            data, _ = self.sock.recvfrom(2048)
        except socket.timeout:
            return
        if len(data) != 1206:
            return
        block_size = 10
        num_blocks = len(data) // block_size
        valid_headers = 0
        for i in range(num_blocks):
            start = i * block_size
            end = start + block_size
            block = data[start:end]
            if len(block) < 6:
                continue
            if block[0] != 0xFF or block[1] != 0xEE:
                continue
            valid_headers += 1
            raw_angle = struct.unpack('<H', block[2:4])[0]
            raw_dist = struct.unpack('<H', block[4:6])[0]
            angle_deg = raw_angle / 100.0
            angle_deg += self.angle_offset_deg
            if self.inverted:
                angle_deg = -angle_deg
            angle_deg = angle_deg % 360.0
            dist_m = raw_dist / 1000.0
            if dist_m < self.range_min or dist_m > self.range_max:
                dist_m = float('inf')
            index = int(angle_deg / self.resolution_deg)
            if index < 0 or index >= self.num_steps:
                continue
            self.frame[index] = dist_m
            if self.last_angle_deg is not None:
                if angle_deg < self.last_angle_deg:
                    self.publish_scan()
            self.last_angle_deg = angle_deg
        if self.debug:
            self.get_logger().info(
                f"packet blocks={num_blocks}, valid_headers={valid_headers}"
            )


    def publish_scan(self):
        now = self.get_clock().now()
        scan_msg = LaserScan()
        scan_msg.header.stamp = now.to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        scan_msg.angle_increment = math.radians(self.resolution_deg)
        if self.last_publish_time is not None:
            dt = (now - self.last_publish_time).nanoseconds / 1e9
            scan_msg.scan_time = float(dt)
        else:
            scan_msg.scan_time = 0.0
        self.last_publish_time = now
        if scan_msg.scan_time > 0.0:
            scan_msg.time_increment = scan_msg.scan_time / float(self.num_steps)
        else:
            scan_msg.time_increment = 0.0
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = list(self.frame)
        scan_msg.intensities = [0.0] * self.num_steps
        self.publisher.publish(scan_msg)
        self.frame = [float('inf')] * self.num_steps
        if self.debug:
            valid = sum(1 for r in scan_msg.ranges if r != float('inf'))
            self.get_logger().info(
                f"published /scan with {valid}/{self.num_steps} valid points, "
                f"scan_time={scan_msg.scan_time:.3f}s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LakiBeam1Driver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
