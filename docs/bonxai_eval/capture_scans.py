"""Save every /lidar/points scan in the map frame, with its sensor origin, for offline benchmarks.

File format (little endian), per scan: uint32 n, float32 origin[3], float32 xyz[n*3].
Points are filtered like mapping.yaml: finite, z in [0.3, 6.0] in the map frame.
usage: capture_scans.py out.bin seconds
"""
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Capture(Node):
    def __init__(self, out):
        super().__init__('scan_capture', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = out
        self.n = 0
        self.pts = 0
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, '/lidar/points', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        try:
            t = self.tf.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)
        tr = t.transform.translation
        o = np.array([tr.x, tr.y, tr.z])
        w = xyz @ quat_to_mat(t.transform.rotation).T + o
        w = w[(w[:, 2] >= 0.3) & (w[:, 2] <= 6.0)].astype(np.float32)
        self.out.write(struct.pack('<I3f', len(w), *o))
        self.out.write(w.tobytes())
        self.n += 1
        self.pts += len(w)


def main():
    rclpy.init()
    with open(sys.argv[1], 'wb') as f:
        node = Capture(f)
        t_end = time.time() + float(sys.argv[2])
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f'{node.n} scans, {node.pts / max(node.n, 1):.0f} points/scan', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
