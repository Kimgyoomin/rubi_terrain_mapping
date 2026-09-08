"""Portable execution tests for RUBI's stamped-TF queue and fusion ordering."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = ROOT / 'elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_node.py'
QUEUE_SOURCE = ROOT / 'elevation_mapping_cupy/elevation_mapping_cupy/stamped_tf_queue.py'
LAUNCH_SOURCE = ROOT / 'rubi_mapping_bringup/launch/rubi_mapping.launch.py'


def load_queue_module():
    spec = importlib.util.spec_from_file_location('stamped_tf_queue_under_test', QUEUE_SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


QUEUE = load_queue_module()
PendingPointCloud = QUEUE.PendingPointCloud
PendingPointCloudQueue = QUEUE.PendingPointCloudQueue


def source_function(path, name, globals_):
    tree = ast.parse(path.read_text())
    definitions = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    for definition in definitions:
        if isinstance(definition, ast.ClassDef):
            candidates = [n for n in definition.body if isinstance(n, ast.FunctionDef)]
        else:
            candidates = [definition]
        for candidate in candidates:
            if candidate.name == name:
                code = ast.Module(body=[candidate], type_ignores=[])
                namespace = dict(globals_)
                exec(compile(ast.fix_missing_locations(code), str(path), 'exec'), namespace)
                return namespace[name]
    raise KeyError(name)


class Logger:
    def __init__(self):
        self.messages = []

    def warning(self, message, **_):
        self.messages.append(('warning', message))

    def fatal(self, message, **_):
        self.messages.append(('fatal', message))


class FakeNode:
    process_pending_pointclouds = source_function(
        NODE_SOURCE,
        'process_pending_pointclouds',
        {
            'np': np,
            'monotonic_time': NS(monotonic=lambda: 10.0),
            'Vector3': lambda **kw: NS(**kw),
            'Quaternion': lambda **kw: NS(**kw),
        },
    )

    def __init__(self, *, sensor_ready=True, base_ready=True, received=9.9, stamp_sec=4):
        self.stamped_tf_queue_enabled = True
        self._epoch_faulted = False
        self._fusion_faulted = False
        self.tf_max_scans_per_tick = 1
        self.tf_wait_timeout = 0.5
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self._pending_clouds = PendingPointCloudQueue(20, 1024)
        self.stamp = NS(sec=stamp_sec, nanosec=20)
        self.msg = NS(header=NS(stamp=self.stamp, frame_id='livox_frame'), data=b'xyz')
        self._pending_clouds.push(PendingPointCloud(
            self.msg, 'lidar', stamp_sec * 1000000000 + 20, received, 3))
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None
        self._last_tf_errors = {}
        self._last_tf_error_stamp_ns = None
        self._queue_wait_samples = []
        self._last_t = None
        self._last_fused_t = None
        self._map_t = None
        self._map_q = None
        self._pointcloud_process_counter = 0
        self._input_stats = {
            'fused': 0, 'timeout_drops': 0, 'invalid_drops': 0,
        }
        self.sensor_ready = sensor_ready
        self.base_ready = base_ready
        self.lookup_calls = []
        self.events = []
        self.logger = Logger()
        self._map = NS(move_to=self._move, input_pointcloud=self._input)

    def get_logger(self):
        return self.logger

    def _observe_ros_clock(self):
        return True

    def _prepare_pointcloud(self, msg, key):
        self.events.append(('prepare', msg.header.stamp, key))
        return np.ones((1, 3), dtype=np.float32), ['x', 'y', 'z']

    def _lookup_original_stamp(self, source, stamp):
        self.lookup_calls.append((source, stamp))
        ready = self.sensor_ready if source == 'livox_frame' else self.base_ready
        if not ready:
            return False, None, f'{source} delayed'
        transform = NS(
            transform=NS(
                translation=NS(
                    x=float(stamp.sec) + (100.0 if source == 'livox_frame' else 0.0),
                    y=2.0,
                    z=3.0,
                ),
                rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        return True, transform, None

    def _transform_to_numpy(self, transform):
        x = transform.transform.translation.x
        theta = x * 0.01
        rot = np.array([
            [np.cos(theta), -np.sin(theta), 0.0],
            [np.sin(theta), np.cos(theta), 0.0],
            [0.0, 0.0, 1.0],
        ])
        return np.array([x, 2, 3]), rot

    def _move(self, trans, rot):
        self.events.append(('move_to', tuple(trans), rot.copy()))

    def _input(self, pts, channels, rot, trans, *_):
        self.events.append(('input_pointcloud', pts.copy(), tuple(channels), rot.copy(), tuple(trans)))

    def _clear_prepared_pending(self):
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None

    def _discard_pending_head(self):
        item = self._pending_clouds.pop()
        if self._prepared_pending_stamp_ns == item.stamp_ns:
            self._clear_prepared_pending()
        return item

    @staticmethod
    def _stamp_text(ns):
        return str(ns)


class QueuePolicyTest(unittest.TestCase):
    def item(self, stamp, size=4):
        return PendingPointCloud(object(), 'lidar', stamp, 1.0, size)

    def test_count_overflow_drops_oldest_deterministically(self):
        queue = PendingPointCloudQueue(2, 100)
        queue.push(self.item(1))
        queue.push(self.item(2))
        result, dropped = queue.push(self.item(3))
        self.assertEqual(result, 'accepted')
        self.assertEqual([x.stamp_ns for x in dropped], [1])
        self.assertEqual([queue.pop().stamp_ns, queue.pop().stamp_ns], [2, 3])

    def test_payload_bound_oversize_duplicate_and_past(self):
        queue = PendingPointCloudQueue(5, 10)
        queue.push(self.item(10, 6))
        _, dropped = queue.push(self.item(20, 6))
        self.assertEqual([x.stamp_ns for x in dropped], [10])
        self.assertEqual(queue.payload_bytes, 6)
        self.assertEqual(queue.push(self.item(20))[0], 'duplicate')
        self.assertEqual(queue.push(self.item(19))[0], 'out_of_order')
        self.assertEqual(queue.push(self.item(0))[0], 'zero_stamp')
        self.assertEqual(queue.push(self.item(30, 11))[0], 'oversize')


class FusionOrderTest(unittest.TestCase):
    def test_ready_transforms_move_then_fuse_at_original_stamp(self):
        node = FakeNode()
        node.process_pending_pointclouds()
        self.assertEqual([e[0] for e in node.events], ['prepare', 'move_to', 'input_pointcloud'])
        self.assertEqual(node.lookup_calls, [('livox_frame', node.stamp), ('base_link', node.stamp)])
        self.assertEqual(node._last_fused_t, node.stamp)
        self.assertEqual(node._input_stats['fused'], 1)
        self.assertEqual(len(node._pending_clouds), 0)

    def test_delayed_tf_waits_without_move_or_success_then_fuses(self):
        node = FakeNode(sensor_ready=False)
        node.process_pending_pointclouds()
        self.assertEqual([e[0] for e in node.events], ['prepare'])
        self.assertIsNone(node._last_fused_t)
        self.assertEqual(len(node._pending_clouds), 1)
        node.sensor_ready = True
        node.process_pending_pointclouds()
        self.assertEqual([e[0] for e in node.events], ['prepare', 'move_to', 'input_pointcloud'])
        self.assertEqual(node._input_stats['fused'], 1)

    def test_delayed_and_immediate_match_on_moving_rotating_trajectory(self):
        for stamp_sec in (4, 5):
            immediate = FakeNode(stamp_sec=stamp_sec)
            immediate.process_pending_pointclouds()
            delayed = FakeNode(stamp_sec=stamp_sec, sensor_ready=False)
            delayed.process_pending_pointclouds()
            delayed.sensor_ready = True
            delayed.process_pending_pointclouds()

            immediate_move, immediate_input = immediate.events[-2:]
            delayed_move, delayed_input = delayed.events[-2:]
            np.testing.assert_allclose(immediate_move[1], delayed_move[1])
            np.testing.assert_allclose(immediate_move[2], delayed_move[2])
            np.testing.assert_allclose(immediate_input[3], delayed_input[3])
            np.testing.assert_allclose(immediate_input[4], delayed_input[4])

        # The synthetic pose changes with stamp, so latest-pose substitution
        # could not satisfy both expected results as a stationary test could.
        first = FakeNode(stamp_sec=4)
        second = FakeNode(stamp_sec=5)
        first.process_pending_pointclouds()
        second.process_pending_pointclouds()
        self.assertNotEqual(first.events[-1][4], second.events[-1][4])

    def test_missing_base_tf_never_moves_or_fuses(self):
        node = FakeNode(base_ready=False)
        node.process_pending_pointclouds()
        self.assertEqual([e[0] for e in node.events], ['prepare'])
        self.assertIsNone(node._last_fused_t)
        self.assertEqual(node._input_stats['fused'], 0)

    def test_timeout_discards_before_late_tf_or_parsing(self):
        node = FakeNode(received=9.0)
        node.process_pending_pointclouds()
        self.assertEqual(node.events, [])
        self.assertEqual(node.lookup_calls, [])
        self.assertEqual(node._input_stats['timeout_drops'], 1)
        self.assertEqual(len(node._pending_clouds), 0)

    def test_fusion_exception_does_not_forge_success(self):
        node = FakeNode()
        node._map.input_pointcloud = lambda *_: (_ for _ in ()).throw(RuntimeError('GPU failed'))
        node.process_pending_pointclouds()
        self.assertTrue(node._fusion_faulted)
        self.assertIsNone(node._last_fused_t)
        self.assertEqual(node._input_stats['fused'], 0)
        self.assertEqual(node.logger.messages[-1][0], 'fatal')

    def test_pose_timer_is_disabled_in_queue_mode(self):
        pose_update = source_function(NODE_SOURCE, 'pose_update', {'np': np})
        node = NS(stamped_tf_queue_enabled=True)
        pose_update(node)


class LookupAndLaunchContractTest(unittest.TestCase):
    def test_queue_and_latest_fallback_are_rejected_together(self):
        initialize = source_function(NODE_SOURCE, '_initialize_input_state', {})
        node = NS(stamped_tf_queue_enabled=True, allow_latest_tf_fallback=True)
        with self.assertRaisesRegex(ValueError, 'cannot both be true'):
            initialize(node)

    def test_strict_lookup_uses_only_original_nonzero_stamp(self):
        calls = []
        stamp = NS(sec=8, nanosec=9)
        node = NS(
            map_frame='map',
            _tf_buffer=NS(lookup_transform=lambda *args: calls.append(args) or 'tf'),
        )
        lookup = source_function(NODE_SOURCE, '_lookup_original_stamp', {})
        self.assertEqual(lookup(node, 'livox_frame', stamp), (True, 'tf', None))
        self.assertEqual(calls, [('map', 'livox_frame', stamp)])

    def test_launch_omission_preserves_yaml_and_explicit_values_override(self):
        optional = source_function(LAUNCH_SOURCE, 'optional_backend_overrides', {})
        self.assertEqual(optional('', ''), {})
        self.assertEqual(
            optional('/override', 'robot'),
            {'subscribers.rubi_lidar.topic_name': '/override', 'base_frame': 'robot'},
        )

    def test_ros_clock_reverse_is_not_confused_with_old_sensor_input(self):
        observe = source_function(NODE_SOURCE, '_observe_ros_clock', {})
        queue = PendingPointCloudQueue(5, 100)
        queue.push(PendingPointCloud(object(), 'lidar', 50, 1.0, 4))
        logger = Logger()
        buffer = NS(cleared=False)
        buffer.clear = lambda: setattr(buffer, 'cleared', True)
        node = NS(
            _last_ros_now_ns=200,
            get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)),
            _pending_clouds=queue,
            _clear_prepared_pending=lambda: None,
            _input_stats={'reset_drops': 0},
            _epoch_faulted=False,
            _last_fused_t=NS(sec=1, nanosec=0),
            _tf_buffer=buffer,
            get_logger=lambda: logger,
        )
        self.assertFalse(observe(node))
        self.assertTrue(node._epoch_faulted)
        self.assertTrue(buffer.cleared)
        self.assertEqual(node._input_stats['reset_drops'], 1)
        self.assertIsNone(node._last_fused_t)
        self.assertEqual(len(queue), 0)


if __name__ == '__main__':
    unittest.main()
