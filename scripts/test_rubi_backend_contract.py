"""Execute the real publisher/TF methods with small ROS/GPU boundary doubles.

These tests require only the Python standard library. They do not exercise CUDA,
DDS, or the full node constructor; the Humble smoke test covers the ROS wrapper.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


SOURCE = Path(__file__).resolve().parents[1] / 'elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_node.py'


def method(name, globals_):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ElevationMappingNode')
    definition = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    code = ast.Module(body=[definition], type_ignores=[])
    namespace = dict(globals_)
    exec(compile(ast.fix_missing_locations(code), str(SOURCE), 'exec'), namespace)
    return namespace[name]


class BackendContract(unittest.TestCase):
    def fake_node(self):
        self.sent = []
        return NS(
            _map_q=object(), _last_t='newer rejected input', _last_fused_t='last fused scan',
            _get_map_center=lambda: [.05, -.1, .7],
            _map_t=NS(x=.069, y=-.081, z=.712),
            _map=NS(resolution=.05, cell_n=162,
                    get_map_with_name_ref=lambda name, output: None),
            map_frame='map', _map_data=[0, .15],
            my_publishers={'raw': {'basic_layers': ['elevation'], 'layers': ['elevation', 'variance']}},
            _numpy_to_multiarray=lambda array, layout: (list(array), layout),
            _publishers_dict={'raw': NS(publish=self.sent.append)},
        )

    @staticmethod
    def message():
        return NS(header=NS(), info=NS(pose=NS(position=NS(), orientation=NS())), data=[])

    def test_publisher_uses_snapped_center_and_absolute_elevation(self):
        node = self.fake_node()
        method('publish_map', {'GridMap': self.message})(node, 'raw')
        msg = self.sent[0]
        self.assertEqual((msg.info.pose.position.x, msg.info.pose.position.y), (.05, -.1))
        self.assertEqual(msg.info.pose.position.z, 0)
        self.assertEqual(msg.info.pose.orientation.w, 1)
        self.assertEqual(msg.info.length_x, 8)
        self.assertEqual(msg.data[0][0], [0, .15])
        self.assertEqual((msg.outer_start_index, msg.inner_start_index), (0, 0))

    def test_failed_input_cannot_refresh_published_stamp(self):
        node = self.fake_node()
        method('publish_map', {'GridMap': self.message})(node, 'raw')
        self.assertEqual(self.sent[0].header.stamp, 'last fused scan')

    def test_no_snapshot_before_first_registered_input(self):
        node = self.fake_node()
        node._last_fused_t = None
        method('publish_map', {'GridMap': self.message})(node, 'raw')
        self.assertEqual(self.sent, [])

    def lookup_case(self, allow_latest):
        class Extrapolation(Exception):
            pass
        calls = []
        def lookup(*args):
            calls.append(args)
            if len(calls) == 1:
                raise Extrapolation()
            return 'latest transform'
        node = NS(allow_latest_tf_fallback=allow_latest,
                  _tf_buffer=NS(lookup_transform=lookup),
                  get_logger=lambda: NS(warning=lambda *a, **kw: None))
        tf = NS(ExtrapolationException=Extrapolation, LookupException=Exception,
                ConnectivityException=Exception)
        result = method('safe_lookup_transform', {
            'tf2_ros': tf, 'tf2': tf, 'rclpy': NS(time=NS(Time=lambda: 'latest'))
        })(node, 'map', 'body', 'scan stamp')
        return calls, result

    def test_rubi_strict_lookup_does_not_retry_latest(self):
        calls, result = self.lookup_case(False)
        self.assertEqual(len(calls), 1)
        self.assertIsNone(result)

    def test_upstream_fallback_remains_opt_in(self):
        calls, result = self.lookup_case(True)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result, 'latest transform')


if __name__ == '__main__':
    unittest.main()
