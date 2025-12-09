import rclpy
from rclpy.node import Node
import socket
import struct
import math
from sensor_msgs.msg import LaserScan


BLOCK_HEADER = 0xEEFF
BLOCK_SIZE = 100
POINTS_PER_BLOCK = 16
POINT_STRIDE = 6
TAIL_BYTES = 6
MAX_GAP_FILL_DEG = 5.0


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
        self.declare_parameter('publish_decimation', 1)

        port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        self.inverted = bool(self.get_parameter('inverted').value)
        self.resolution_deg = float(self.get_parameter('resolution_deg').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.publish_decimation = max(1, int(self.get_parameter('publish_decimation').value))
        self.num_steps = int(360.0 / self.resolution_deg)
        self.frame = [float('inf')] * self.num_steps
        self.intensity_frame = [0.0] * self.num_steps
        self.last_angle_deg = None
        self.last_block_angle_raw = None
        self.rotation_base = 0.0
        self.last_filled_index = None
        self.last_filled_value = float('inf')
        self.last_filled_intensity = 0.0
        self.last_publish_time = None
        self.pending_block = None
        self.last_block_delta_deg = None
        self.scan_sequence = 0
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
        blocks = self._decode_blocks(data)
        if not blocks:
            return
        processed_blocks = 0
        if self.pending_block is not None:
            self._process_block(self.pending_block, blocks[0]['angle_deg'])
            processed_blocks += 1
            self.pending_block = None
        for idx in range(len(blocks) - 1):
            self._process_block(blocks[idx], blocks[idx + 1]['angle_deg'])
            processed_blocks += 1
        self.pending_block = blocks[-1]
        if self.debug:
            self.get_logger().info(
                f"packet len={len(data)}, decoded_blocks={len(blocks)}, "
                f"processed={processed_blocks}"
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
        scan_msg.intensities = list(self.intensity_frame)
        self.publisher.publish(scan_msg)
        self._reset_scan_buffers()
        if self.debug:
            valid = sum(1 for r in scan_msg.ranges if r != float('inf'))
            self.get_logger().info(
                f"published /scan with {valid}/{self.num_steps} valid points, "
                f"scan_time={scan_msg.scan_time:.3f}s"
            )

    def _reset_scan_buffers(self):
        self.frame = [float('inf')] * self.num_steps
        self.intensity_frame = [0.0] * self.num_steps
        self.last_filled_index = None
        self.last_filled_value = float('inf')
        self.last_filled_intensity = 0.0

    def _complete_scan(self):
        should_publish = (self.scan_sequence % self.publish_decimation) == 0
        if should_publish:
            self.publish_scan()
        else:
            self._reset_scan_buffers()
        self.scan_sequence += 1

    def _decode_blocks(self, data):
        usable_length = len(data) - TAIL_BYTES if len(data) > TAIL_BYTES else len(data)
        view = memoryview(data)
        blocks = []
        for start in range(0, usable_length, BLOCK_SIZE):
            block = view[start:start + BLOCK_SIZE]
            if len(block) < BLOCK_SIZE:
                break
            header = struct.unpack_from('<H', block, 0)[0]
            if header != BLOCK_HEADER:
                continue
            raw_angle = struct.unpack_from('<H', block, 2)[0]
            ranges = []
            intensities = []
            for channel in range(POINTS_PER_BLOCK):
                offset = 4 + channel * POINT_STRIDE
                raw_dist = struct.unpack_from('<H', block, offset)[0]
                raw_intensity = struct.unpack_from('<H', block, offset + 2)[0]
                ranges.append(raw_dist / 1000.0)
                intensities.append(float(raw_intensity))
            blocks.append({
                'angle_deg': raw_angle / 100.0,
                'ranges': ranges,
                'intensities': intensities,
            })
        return blocks

    def _process_block(self, block, next_angle_deg):
        ranges = block.get('ranges', [])
        intensities = block.get('intensities', [])
        if not ranges:
            return
        current_raw_angle = block['angle_deg']
        if self.last_block_angle_raw is not None:
            delta_raw = current_raw_angle - self.last_block_angle_raw
            if delta_raw < -20.0:
                self._complete_scan()
                self.rotation_base += 360.0
                self.last_angle_deg = None
        self.last_block_angle_raw = current_raw_angle
        current_angle = current_raw_angle
        delta = (next_angle_deg - current_raw_angle) % 360.0
        if delta <= 0.0:
            if self.last_block_delta_deg is not None:
                delta = self.last_block_delta_deg
            else:
                delta = self.resolution_deg
        self.last_block_delta_deg = delta
        channel_increment = delta / float(len(ranges))
        for idx, raw_dist_m in enumerate(ranges):
            intensity = intensities[idx] if idx < len(intensities) else 0.0
            unwrapped_angle = self.rotation_base + current_angle + idx * channel_increment
            angle_deg = unwrapped_angle + self.angle_offset_deg
            if self.inverted:
                angle_deg = -angle_deg
            angle_deg = angle_deg % 360.0
            if raw_dist_m <= 0.0:
                dist_m = float('inf')
            else:
                if raw_dist_m < self.range_min or raw_dist_m > self.range_max:
                    dist_m = float('inf')
                else:
                    dist_m = raw_dist_m
            index = int(round(angle_deg / self.resolution_deg)) % self.num_steps
            if index < 0 or index >= self.num_steps:
                continue
            self._fill_gap(index, dist_m, intensity)
            self.frame[index] = dist_m
            self.intensity_frame[index] = intensity
            self.last_angle_deg = angle_deg

    def _fill_gap(self, current_index, current_value, current_intensity):
        if self.last_filled_index is None:
            self.last_filled_index = current_index
            self.last_filled_value = current_value
            self.last_filled_intensity = current_intensity
            return
        gap = (current_index - self.last_filled_index) % self.num_steps
        if gap > 1 and gap < self.num_steps - 1:
            gap_deg = gap * self.resolution_deg
            if gap_deg > MAX_GAP_FILL_DEG:
                self.last_filled_index = current_index
                self.last_filled_value = current_value
                self.last_filled_intensity = current_intensity
                return
            if math.isfinite(self.last_filled_value) and math.isfinite(current_value):
                for step in range(1, gap):
                    ratio = step / float(gap)
                    interp = self.last_filled_value + (current_value - self.last_filled_value) * ratio
                    interp_intensity = self.last_filled_intensity + (current_intensity - self.last_filled_intensity) * ratio
                    idx = (self.last_filled_index + step) % self.num_steps
                    self.frame[idx] = interp
                    self.intensity_frame[idx] = interp_intensity
        self.last_filled_index = current_index
        self.last_filled_value = current_value
        self.last_filled_intensity = current_intensity


def main(args=None):
    rclpy.init(args=args)
    node = LakiBeam1Driver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
