"""Humble DDS smoke test of the compiled wrapper; no robot, CUDA or GPU needed."""
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import tempfile
import time
import unittest

import rclpy
from grid_map_msgs.msg import GridMap
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger


class WrapperROS(unittest.TestCase):
    def test_persistence_and_services(self):
        with tempfile.TemporaryDirectory(prefix='rubi_wrapper_test_') as directory:
            os.environ['ROS_DOMAIN_ID'] = '173'
            rclpy.init()
            node = rclpy.create_node('rubi_wrapper_test')
            received = []
            pub = node.create_publisher(GridMap, '/rubi_test/local', 2)
            sub = node.create_subscription(PointCloud2, '/rubi_test/global', received.append, 2)
            del sub  # node retains ownership
            log_path = Path(directory) / 'node.log'
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    'ros2', 'run', 'rubi_global_heightmap_wrapper', 'global_heightmap_node',
                    '--ros-args', '-p', 'input_topic:=/rubi_test/local',
                    '-p', 'output_topic:=/rubi_test/global',
                    '-p', 'origin_x:=-0.1', '-p', 'origin_y:=-0.1',
                    '-p', 'length_x:=0.4', '-p', 'length_y:=0.2',
                    '-p', 'publish_fps:=10.0',
                    '-p', f'output_directory:={directory}',
                    '-p', f'load_path:={directory}/restore.rghm',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    def pump(predicate, message=None, seconds=10):
                        end = time.monotonic() + seconds
                        while time.monotonic() < end:
                            if process.poll() is not None:
                                self.fail('Wrapper exited: ' + log_path.read_text())
                            if message is not None:
                                pub.publish(message)
                            rclpy.spin_once(node, timeout_sec=.05)
                            if predicate():
                                return
                        self.fail('Timed out: ' + log_path.read_text())

                    def snapshot(stamp, center_x=0.0):
                        msg = GridMap()
                        msg.header.frame_id = 'map'
                        msg.header.stamp.sec = stamp
                        msg.info.resolution = .05
                        msg.info.length_x = .1
                        msg.info.length_y = .1
                        msg.info.pose.position.x = center_x
                        msg.info.pose.orientation.w = 1.0
                        msg.layers = ['elevation', 'variance']
                        msg.basic_layers = ['elevation']
                        for values in ([0., .05, .15, .20], [.001] * 4):
                            layer = Float32MultiArray()
                            layer.layout.dim = [
                                MultiArrayDimension(label='column_index', size=2, stride=4),
                                MultiArrayDimension(label='row_index', size=2, stride=2),
                            ]
                            layer.data = values
                            msg.data.append(layer)
                        return msg

                    def points(msg):
                        self.assertEqual(msg.height, 1)
                        self.assertEqual(msg.point_step, 12)
                        self.assertEqual(msg.row_step, 12 * msg.width)
                        self.assertEqual([f.name for f in msg.fields], ['x', 'y', 'z'])
                        code = '>fff' if msg.is_bigendian else '<fff'
                        return [struct.unpack_from(code, msg.data, i * 12) for i in range(msg.width)]

                    def service(name):
                        client = node.create_client(Trigger, '/rubi/global_elevation/' + name)
                        self.assertTrue(client.wait_for_service(timeout_sec=5))
                        future = client.call_async(Trigger.Request())
                        pump(future.done)
                        response = future.result()
                        self.assertTrue(response.success, response.message)
                        node.destroy_client(client)
                        return response

                    pump(lambda: any(m.header.stamp.sec == 1 for m in received), snapshot(1))
                    first = next(m for m in received if m.header.stamp.sec == 1)
                    self.assertEqual(first.width, 4)
                    p = points(first)
                    self.assertAlmostEqual(p[0][0], -.025, places=5)
                    self.assertAlmostEqual(p[0][1], -.025, places=5)
                    self.assertAlmostEqual(p[1][2], .15, places=5)
                    self.assertAlmostEqual(p[2][2], .05, places=5)

                    pump(lambda: any(m.header.stamp.sec == 2 for m in received), snapshot(2, .1))
                    second = next(m for m in received if m.header.stamp.sec == 2)
                    self.assertEqual(second.width, 8)
                    points(second)
                    bad = snapshot(3, .125)  # half-cell phase, not an interpolation request
                    deadline = time.monotonic() + .6
                    while time.monotonic() < deadline:
                        pub.publish(bad)
                        rclpy.spin_once(node, timeout_sec=.05)
                    self.assertFalse(any(m.header.stamp.sec == 3 for m in received))

                    path = Path(service('save_map').message)
                    self.assertTrue((path / 'surface.pcd').exists())
                    shutil.copyfile(path / 'map.rghm', Path(directory) / 'restore.rghm')
                    service('clear_map')
                    received.clear()
                    pump(lambda: any(m.header.stamp.sec == 4 for m in received), snapshot(4))
                    self.assertEqual(next(m for m in received if m.header.stamp.sec == 4).width, 4)
                    service('load_map')
                    received.clear()
                    pump(lambda: any(m.header.stamp.sec == 2 and m.width == 8 for m in received))
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()

    def test_clock_reset_stops_global_publication(self):
        with tempfile.TemporaryDirectory(prefix='rubi_wrapper_clock_test_') as directory:
            os.environ['ROS_DOMAIN_ID'] = '174'
            rclpy.init()
            node = rclpy.create_node('rubi_wrapper_clock_test')
            received = []
            local_pub = node.create_publisher(GridMap, '/rubi_clock/local', 2)
            clock_pub = node.create_publisher(Clock, '/clock', 10)
            sub = node.create_subscription(PointCloud2, '/rubi_clock/global', received.append, 2)
            del sub
            log_path = Path(directory) / 'node.log'
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    'ros2', 'run', 'rubi_global_heightmap_wrapper', 'global_heightmap_node',
                    '--ros-args', '-p', 'use_sim_time:=true',
                    '-p', 'input_topic:=/rubi_clock/local',
                    '-p', 'output_topic:=/rubi_clock/global',
                    '-p', 'origin_x:=-0.1', '-p', 'origin_y:=-0.1',
                    '-p', 'length_x:=0.2', '-p', 'length_y:=0.2',
                    '-p', 'publish_fps:=10.0',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    def clock(sec):
                        msg = Clock()
                        msg.clock.sec = sec
                        return msg

                    snapshot = GridMap()
                    snapshot.header.frame_id = 'map'
                    snapshot.header.stamp.sec = 9
                    snapshot.info.resolution = .05
                    snapshot.info.length_x = .1
                    snapshot.info.length_y = .1
                    snapshot.info.pose.orientation.w = 1.0
                    snapshot.layers = ['elevation', 'variance']
                    snapshot.basic_layers = ['elevation']
                    for values in ([0., .05, .15, .20], [.001] * 4):
                        layer = Float32MultiArray()
                        layer.layout.dim = [
                            MultiArrayDimension(label='column_index', size=2, stride=4),
                            MultiArrayDimension(label='row_index', size=2, stride=2),
                        ]
                        layer.data = values
                        snapshot.data.append(layer)

                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline and not received:
                        if process.poll() is not None:
                            self.fail('Wrapper exited: ' + log_path.read_text())
                        clock_pub.publish(clock(10))
                        local_pub.publish(snapshot)
                        rclpy.spin_once(node, timeout_sec=.05)
                    self.assertTrue(received, log_path.read_text())

                    received.clear()
                    deadline = time.monotonic() + .6
                    while time.monotonic() < deadline:
                        clock_pub.publish(clock(5))
                        local_pub.publish(snapshot)
                        rclpy.spin_once(node, timeout_sec=.05)
                    self.assertEqual(received, [])
                    self.assertIn('ROS time moved backwards', log_path.read_text())
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
