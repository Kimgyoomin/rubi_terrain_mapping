#!/usr/bin/env python3
import math
import message_filters
import numpy as np
import os
import time as monotonic_time
from pathlib import Path
from functools import partial
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from elevation_map_msgs.msg import ChannelInfo
import ros2_numpy as rnp
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf_transformations import quaternion_matrix
import tf2_ros
import tf2_py as tf2
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from rclpy.serialization import serialize_message, deserialize_message
from grid_map_msgs.msg import GridMap
from grid_map_msgs.srv import SetGridMap, ProcessFile
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout as MAL
from std_msgs.msg import MultiArrayDimension as MAD
import rosbag2_py
from elevation_mapping_cupy import ElevationMap, Parameter
from elevation_mapping_cupy.elevation_mapping import GridGeometry
from elevation_mapping_cupy.gridmap_utils import encode_layer_to_multiarray, decode_multiarray_to_rows_cols
from elevation_mapping_cupy.stamped_tf_queue import PendingPointCloud, PendingPointCloudQueue

PDC_DATATYPE = {
    "1": np.int8,
    "2": np.uint8,
    "3": np.int16,
    "4": np.uint16,
    "5": np.int32,
    "6": np.uint32,
    "7": np.float32,
    "8": np.float64,
}

def _pointcloud2_xyz_f32(msg: PointCloud2) -> np.ndarray:
    """
    Convert a PointCloud2 into an (N,3) float32 numpy array for fields (x,y,z).

    Supported (fail-loudly):
      - little-endian clouds
      - fields x,y,z present and FLOAT32

    This intentionally does not support arbitrary field layouts or RGB/semantic channels.
    """
    if msg.is_bigendian:
        raise ValueError("PointCloud2 big-endian is not supported.")

    want = {"x", "y", "z"}
    fields = {f.name: f for f in msg.fields}
    missing = want.difference(fields.keys())
    if missing:
        raise ValueError(f"PointCloud2 is missing required fields: {sorted(missing)}")

    for name in ("x", "y", "z"):
        f = fields[name]
        if f.datatype != PointField.FLOAT32 or f.count != 1:
            raise ValueError(
                f"PointCloud2 field '{name}' must be FLOAT32 count=1, got datatype={f.datatype} count={f.count}"
            )

    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": (np.float32, np.float32, np.float32),
            "offsets": (fields["x"].offset, fields["y"].offset, fields["z"].offset),
            "itemsize": msg.point_step,
        }
    )
    arr = np.frombuffer(msg.data, dtype=dtype)
    pts = np.stack((arr["x"], arr["y"], arr["z"]), axis=-1).astype(np.float32, copy=False)

    # Do not trust is_dense blindly: malformed drivers occasionally mark clouds
    # dense while still carrying NaN/Inf samples.
    good = np.isfinite(pts).all(axis=1)
    pts = pts[good]
    return pts

class ElevationMappingNode(Node):
    def __init__(self):
        super().__init__(
            'elevation_mapping_node',
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=False
        )

        self.root = get_package_share_directory("elevation_mapping_cupy")
        weight_file = os.path.join(self.root, "config/core/weights.dat")
        plugin_config_file = os.path.join(self.root, "config/core/plugin_config.yaml")

        # Initialize parameters with some defaults
        self.param = Parameter(
            use_chainer=False,
            weight_file=weight_file,
            plugin_config_file=plugin_config_file
        )

        # Read ROS parameters (including YAML)
        self.initialize_ros()
        self.set_param_values_from_ros()

        # Overwrite subscriber_cfg from loaded YAML
        self.param.subscriber_cfg = self.my_subscribers

        self._last_t = None
        self._last_fused_t = None
        self._initialize_input_state()
        self.initialize_elevation_mapping()
        self.register_subscribers()
        self.register_publishers()
        self.register_timers()
        self.register_services()

    def initialize_elevation_mapping(self) -> None:
        self.param.update()
        self._pointcloud_process_counter = 0
        self._image_process_counter = 0
        self._map = ElevationMap(self.param)
        self._map_data = np.zeros(
            (self._map.cell_n - 2, self._map.cell_n - 2), dtype=np.float32
        )
        self.get_logger().info(f"Initialized map with length: {self._map.map_length}, resolution: {self._map.resolution}, cells: {self._map.cell_n}")

        self._map_q = None
        self._map_t = None

    def initialize_ros(self) -> None:
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # YAML overrides are auto-declared. Explicit defaults keep older configs
        # (which predate the queue) usable on the inherited non-queued path.
        queue_defaults = {
            'stamped_tf_queue_enabled': False,
            'tf_wait_timeout': 0.5,
            'tf_queue_size': 20,
            'tf_queue_max_bytes': 67108864,
            'tf_retry_period': 0.02,
            'tf_max_scans_per_tick': 1,
            'allow_latest_tf_fallback': True,
        }
        for name, default in queue_defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        self.get_ros_params()

    def get_ros_params(self) -> None:
        self.use_chainer = self.get_parameter('use_chainer').get_parameter_value().bool_value
        self.initialize_frame_id = self.get_parameter(
            'initialize_frame_id'
        ).get_parameter_value().string_array_value
        self.initialize_tf_offset = self.get_parameter('initialize_tf_offset').get_parameter_value().double_array_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.allow_latest_tf_fallback = self.get_parameter(
            'allow_latest_tf_fallback'
        ).get_parameter_value().bool_value
        self.stamped_tf_queue_enabled = self.get_parameter(
            'stamped_tf_queue_enabled'
        ).get_parameter_value().bool_value
        self.tf_wait_timeout = self.get_parameter('tf_wait_timeout').get_parameter_value().double_value
        self.tf_queue_size = self.get_parameter('tf_queue_size').get_parameter_value().integer_value
        self.tf_queue_max_bytes = self.get_parameter('tf_queue_max_bytes').get_parameter_value().integer_value
        self.tf_retry_period = self.get_parameter('tf_retry_period').get_parameter_value().double_value
        self.tf_max_scans_per_tick = self.get_parameter(
            'tf_max_scans_per_tick'
        ).get_parameter_value().integer_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.corrected_map_frame = self.get_parameter('corrected_map_frame').get_parameter_value().string_value
        self.initialize_method = self.get_parameter('initialize_method').get_parameter_value().string_value
        self.position_lowpass_alpha = self.get_parameter('position_lowpass_alpha').get_parameter_value().double_value
        self.orientation_lowpass_alpha = self.get_parameter('orientation_lowpass_alpha').get_parameter_value().double_value
        self.recordable_fps = self.get_parameter('recordable_fps').get_parameter_value().double_value
        self.update_variance_fps = self.get_parameter('update_variance_fps').get_parameter_value().double_value
        self.time_interval = self.get_parameter('time_interval').get_parameter_value().double_value
        self.update_pose_fps = self.get_parameter('update_pose_fps').get_parameter_value().double_value
        self.initialize_tf_grid_size = self.get_parameter('initialize_tf_grid_size').get_parameter_value().double_value
        self.map_acquire_fps = self.get_parameter('map_acquire_fps').get_parameter_value().double_value
        self.publish_statistics_fps = self.get_parameter('publish_statistics_fps').get_parameter_value().double_value
        self.enable_pointcloud_publishing = self.get_parameter('enable_pointcloud_publishing').get_parameter_value().bool_value
        self.enable_normal_arrow_publishing = self.get_parameter('enable_normal_arrow_publishing').get_parameter_value().bool_value
        self.enable_drift_corrected_TF_publishing = self.get_parameter('enable_drift_corrected_TF_publishing').get_parameter_value().bool_value
        self.use_initializer_at_start = self.get_parameter('use_initializer_at_start').get_parameter_value().bool_value
        subscribers_params = self.get_parameters_by_prefix('subscribers')
        self.my_subscribers = {}
        for param_name, param_value in subscribers_params.items():
            parts = param_name.split('.')
            if len(parts) >= 2:
                sub_key, sub_param = parts[:2]
                if sub_key not in self.my_subscribers:
                    self.my_subscribers[sub_key] = {}
                self.my_subscribers[sub_key][sub_param] = param_value.value
        publishers_params = self.get_parameters_by_prefix('publishers')
        self.my_publishers = {}
        for param_name, param_value in publishers_params.items():
            parts = param_name.split('.')
            if len(parts) >= 2:
                pub_key, pub_param = parts[:2]
                if pub_key not in self.my_publishers:
                    self.my_publishers[pub_key] = {}
                self.my_publishers[pub_key][pub_param] = param_value.value

    def _initialize_input_state(self) -> None:
        if self.stamped_tf_queue_enabled and self.allow_latest_tf_fallback:
            raise ValueError(
                "stamped_tf_queue_enabled and allow_latest_tf_fallback cannot both be true"
            )
        if self.tf_wait_timeout <= 0.0 or self.tf_retry_period <= 0.0:
            raise ValueError("tf_wait_timeout and tf_retry_period must be positive")
        if self.tf_max_scans_per_tick <= 0:
            raise ValueError("tf_max_scans_per_tick must be positive")

        if self.stamped_tf_queue_enabled:
            pointclouds = [
                (key, cfg) for key, cfg in self.my_subscribers.items()
                if cfg.get('data_type') == 'pointcloud'
            ]
            unsupported = [
                key for key, cfg in self.my_subscribers.items()
                if cfg.get('data_type') != 'pointcloud' or cfg.get('channels')
            ]
            if len(pointclouds) != 1 or unsupported:
                raise ValueError(
                    "stamped TF queue supports exactly one XYZ-only pointcloud subscriber; "
                    f"pointclouds={len(pointclouds)}, unsupported={unsupported}"
                )

        self._pending_clouds = PendingPointCloudQueue(
            self.tf_queue_size, self.tf_queue_max_bytes
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_ros_now_ns = None
        self._epoch_faulted = False
        self._fusion_faulted = False
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None
        self._queue_wait_samples = []
        self._last_tf_errors = {}
        self._last_tf_error_stamp_ns = None
        self._last_received_stamp_ns = None
        self._input_stats = {
            'received': 0,
            'fused': 0,
            'timeout_drops': 0,
            'overflow_drops': 0,
            'invalid_drops': 0,
            'duplicate_out_of_order_drops': 0,
            'reset_drops': 0,
        }

        topics = [
            str(cfg.get('topic_name', '')) for cfg in self.my_subscribers.values()
            if cfg.get('data_type') == 'pointcloud'
        ]
        self.get_logger().info(
            "Input configuration: topics=%s map_frame='%s' base_frame='%s' "
            "stamped_tf_queue_enabled=%s allow_latest_tf_fallback=%s"
            % (topics, self.map_frame, self.base_frame,
               self.stamped_tf_queue_enabled, self.allow_latest_tf_fallback)
        )

    def set_param_values_from_ros(self):
        # Assign to self.param so it won't use defaults. This is research code: crash loudly if
        # a required parameter is missing or mistyped.
        self.param.use_chainer = self.use_chainer
        self.param.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.param.map_length = self.get_parameter('map_length').get_parameter_value().double_value
        self.param.sensor_noise_factor = self.get_parameter('sensor_noise_factor').get_parameter_value().double_value
        self.param.mahalanobis_thresh = self.get_parameter('mahalanobis_thresh').get_parameter_value().double_value
        self.param.outlier_variance = self.get_parameter('outlier_variance').get_parameter_value().double_value
        self.param.drift_compensation_variance_inlier = self.get_parameter(
            'drift_compensation_variance_inlier'
        ).get_parameter_value().double_value
        self.param.checker_layer = self.get_parameter('checker_layer').get_parameter_value().string_value
        self.param.max_drift = self.get_parameter('max_drift').get_parameter_value().double_value
        self.param.drift_compensation_alpha = self.get_parameter(
            'drift_compensation_alpha'
        ).get_parameter_value().double_value
        self.param.time_variance = self.get_parameter('time_variance').get_parameter_value().double_value
        self.param.max_variance = self.get_parameter('max_variance').get_parameter_value().double_value
        self.param.initial_variance = self.get_parameter('initial_variance').get_parameter_value().double_value
        self.param.initialized_variance = self.get_parameter(
            'initialized_variance'
        ).get_parameter_value().double_value
        self.param.traversability_inlier = self.get_parameter(
            'traversability_inlier'
        ).get_parameter_value().double_value
        self.param.dilation_size = self.get_parameter('dilation_size').get_parameter_value().integer_value
        self.param.dilation_size_initialize = self.get_parameter(
            'dilation_size_initialize'
        ).get_parameter_value().integer_value
        self.param.wall_num_thresh = self.get_parameter('wall_num_thresh').get_parameter_value().integer_value
        self.param.min_height_drift_cnt = self.get_parameter(
            'min_height_drift_cnt'
        ).get_parameter_value().integer_value
        self.param.position_noise_thresh = self.get_parameter(
            'position_noise_thresh'
        ).get_parameter_value().double_value
        self.param.orientation_noise_thresh = self.get_parameter(
            'orientation_noise_thresh'
        ).get_parameter_value().double_value
        self.param.min_valid_distance = self.get_parameter(
            'min_valid_distance'
        ).get_parameter_value().double_value
        self.param.max_height_range = self.get_parameter(
            'max_height_range'
        ).get_parameter_value().double_value
        self.param.ramped_height_range_a = self.get_parameter(
            'ramped_height_range_a'
        ).get_parameter_value().double_value
        self.param.ramped_height_range_b = self.get_parameter(
            'ramped_height_range_b'
        ).get_parameter_value().double_value
        self.param.ramped_height_range_c = self.get_parameter(
            'ramped_height_range_c'
        ).get_parameter_value().double_value
        self.param.max_ray_length = self.get_parameter('max_ray_length').get_parameter_value().double_value
        self.param.cleanup_step = self.get_parameter('cleanup_step').get_parameter_value().double_value
        self.param.cleanup_cos_thresh = self.get_parameter(
            'cleanup_cos_thresh'
        ).get_parameter_value().double_value
        self.param.safe_thresh = self.get_parameter('safe_thresh').get_parameter_value().double_value
        self.param.safe_min_thresh = self.get_parameter('safe_min_thresh').get_parameter_value().double_value
        self.param.max_unsafe_n = self.get_parameter('max_unsafe_n').get_parameter_value().integer_value
        self.param.overlap_clear_range_xy = self.get_parameter(
            'overlap_clear_range_xy'
        ).get_parameter_value().double_value
        self.param.overlap_clear_range_z = self.get_parameter(
            'overlap_clear_range_z'
        ).get_parameter_value().double_value
        self.param.enable_edge_sharpen = self.get_parameter(
            'enable_edge_sharpen'
        ).get_parameter_value().bool_value
        self.param.enable_visibility_cleanup = self.get_parameter(
            'enable_visibility_cleanup'
        ).get_parameter_value().bool_value
        self.param.enable_drift_compensation = self.get_parameter(
            'enable_drift_compensation'
        ).get_parameter_value().bool_value
        self.param.enable_overlap_clearance = self.get_parameter(
            'enable_overlap_clearance'
        ).get_parameter_value().bool_value
        self.param.use_only_above_for_upper_bound = self.get_parameter(
            'use_only_above_for_upper_bound'
        ).get_parameter_value().bool_value

        mask_param = self.get_parameter('masked_replace_service_mask_layer_name').get_parameter_value().string_value
        topic_param = self.get_parameter('save_map_default_topic').get_parameter_value().string_value
        storage_param = self.get_parameter('save_map_storage_id').get_parameter_value().string_value
        service_ns_param = self.get_parameter('service_namespace').get_parameter_value().string_value

        if not mask_param:
            raise ValueError("masked_replace_service_mask_layer_name must be a non-empty string")
        if not topic_param:
            raise ValueError("save_map_default_topic must be a non-empty string")
        if not storage_param:
            raise ValueError("save_map_storage_id must be a non-empty string")
        if not service_ns_param:
            raise ValueError("service_namespace must be a non-empty string")

        self.masked_replace_mask_layer_name = mask_param
        self.save_map_default_topic = topic_param
        self.save_map_storage_id = storage_param
        self.service_namespace = self._normalize_namespace(service_ns_param)

    def register_subscribers(self) -> None:
        self._pointcloud_subs = {}
        self._image_syncs = {}
        self._image_filter_subs = {}
        self._channel_info_subs = {}
        self._image_channels = {}

        if any(config.get("data_type") == "image" for config in self.my_subscribers.values()):
            self.cv_bridge = CvBridge()

        for key, config in self.my_subscribers.items():
            data_type = config.get("data_type")
            if data_type == "image":
                topic_name = config.get("topic_name")
                camera_info_topic_name = config.get(
                    "camera_info_topic_name",
                    config.get("topic_name_camera_info"),
                )
                if not topic_name:
                    raise ValueError(f"Image subscriber '{key}' is missing required key 'topic_name'.")
                if not camera_info_topic_name:
                    raise ValueError(
                        f"Image subscriber '{key}' is missing required key 'camera_info_topic_name'."
                    )

                camera_sub = message_filters.Subscriber(self, Image, topic_name)
                camera_info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic_name)
                image_sync = message_filters.ApproximateTimeSynchronizer(
                    [camera_sub, camera_info_sub],
                    queue_size=10,
                    slop=0.5,
                )
                image_sync.registerCallback(partial(self.image_callback, sub_key=key))
                self._image_filter_subs[key] = [camera_sub, camera_info_sub]
                self._image_syncs[key] = image_sync

                channel_info_topic_name = config.get("channel_info_topic_name")
                if channel_info_topic_name:
                    self._channel_info_subs[key] = self.create_subscription(
                        ChannelInfo,
                        channel_info_topic_name,
                        partial(self.channel_info_callback, sub_key=key),
                        10,
                    )
                continue

            if data_type != "pointcloud":
                raise ValueError(
                    f"Unsupported subscriber data_type='{data_type}' for '{key}'. "
                    "Supported: pointcloud and image."
                )

            topic_name = config.get("topic_name")
            if not topic_name:
                raise ValueError(f"Subscriber '{key}' is missing required key 'topic_name'.")

            # Use sensor data QoS (BEST_EFFORT) for point clouds
            qos_profile = QoSPresetProfiles.get_from_short_key("sensor_data")
            self._pointcloud_subs[key] = self.create_subscription(
                PointCloud2,
                topic_name,
                partial(self.pointcloud_callback, sub_key=key),
                qos_profile,
            )

    def channel_info_callback(self, msg: ChannelInfo, sub_key: str) -> None:
        self._image_channels[sub_key] = list(msg.channels)

    def resolve_image_channels(self, sub_key: str) -> List[str]:
        configured_channels = self.param.subscriber_cfg[sub_key].get("channels", [])
        if configured_channels:
            return configured_channels

        live_channels = self._image_channels.get(sub_key, [])
        if live_channels:
            return live_channels

        self.get_logger().warning(
            (
                f"Image subscriber '{sub_key}' has no resolved channels yet. "
                "Configure 'channels' or wait for ChannelInfo."
            ),
            throttle_duration_sec=5.0,
        )
        return []

    def register_publishers(self) -> None:
        self._publishers_dict = {}
        self._publishers_timers = []

        for pub_key, pub_config in self.my_publishers.items():
            topic_name = f"/{self.get_name()}/{pub_key}"
            publisher = self.create_publisher(GridMap, topic_name, 10)
            self._publishers_dict[pub_key] = publisher

            fps = pub_config.get("fps", 1.0)
            timer = self.create_timer(
                1.0 / fps,
                partial(self.publish_map, key=pub_key)
            )
            self._publishers_timers.append(timer)

    def register_timers(self) -> None:
        self.time_pose_update = self.create_timer(
            0.1,
            self.pose_update
        )
        self.timer_variance = self.create_timer(
            1.0 / self.update_variance_fps,
            self.update_variance
        )
        self.timer_time = self.create_timer(
            self.time_interval,
            self.update_time
        )
        # Queue residence timeout and retry cadence are wall/steady-time based,
        # so Gazebo pause cannot retain scans forever. TF lookup still uses the
        # original ROS timestamp carried by each PointCloud2.
        self.timer_tf_queue = self.create_timer(
            self.tf_retry_period,
            self.process_pending_pointclouds,
            clock=self._steady_clock,
        )
        self.timer_input_statistics = self.create_timer(
            1.0 / max(self.publish_statistics_fps, 0.1),
            self.publish_input_statistics,
            clock=self._steady_clock,
        )

    def register_services(self) -> None:
        service_masked = self._resolve_service_name('masked_replace')
        service_save = self._resolve_service_name('save_map')
        service_load = self._resolve_service_name('load_map')

        self._srv_masked_replace = self.create_service(
            SetGridMap,
            service_masked,
            self.handle_masked_replace
        )
        self._srv_save_map = self.create_service(
            ProcessFile,
            service_save,
            self.handle_save_map
        )
        self._srv_load_map = self.create_service(
            ProcessFile,
            service_load,
            self.handle_load_map
        )

    def publish_map(self, key: str) -> None:
        if (self._map_q is None or self._last_fused_t is None or
                self._epoch_faulted or self._fusion_faulted):
            return
        center = self._get_map_center()
        gm = GridMap()
        gm.header.frame_id = self.map_frame
        # A TF-rejected input must not refresh the timestamp of an old map.
        gm.header.stamp = self._last_fused_t
        gm.info.resolution = self._map.resolution
        actual_map_length = (self._map.cell_n - 2) * self._map.resolution
        gm.info.length_x = actual_map_length
        gm.info.length_y = actual_map_length
        # move_to snaps XY to whole cells. The continuous robot position is not
        # the grid center. Elevation already includes the absolute center Z.
        gm.info.pose.position.x = float(center[0])
        gm.info.pose.position.y = float(center[1])
        gm.info.pose.position.z = 0.0

        gm.info.pose.orientation.x = 0.0
        gm.info.pose.orientation.y = 0.0
        gm.info.pose.orientation.z = 0.0
        gm.info.pose.orientation.w = 1.0
        gm.layers = []
        gm.basic_layers = self.my_publishers[key]["basic_layers"]

        for layer in self.my_publishers[key].get("layers", []):
            gm.layers.append(layer)
            self._map.get_map_with_name_ref(layer, self._map_data)
            # After fixing CUDA kernels and removing flips in elevation_mapping.py, no flip needed here
            map_data_for_gridmap = self._map_data
            gm.data.append(self._numpy_to_multiarray(map_data_for_gridmap, layout="gridmap_column"))

        gm.outer_start_index = 0
        gm.inner_start_index = 0
        self._publishers_dict[key].publish(gm)

    def handle_masked_replace(self, request, response):
        try:
            layer_arrays, geometry = self._grid_map_to_numpy(request.map)
            mask = layer_arrays.pop(self.masked_replace_mask_layer_name, None)
            if not layer_arrays:
                raise ValueError("Provide at least one data layer to update.")
            self._map.apply_masked_replace(layer_arrays, mask, geometry)
            self._republish_all_once()
            self.get_logger().info(f"masked_replace updated {len(layer_arrays)} layer(s).")
        except Exception as exc:
            self.get_logger().error(f"masked_replace failed: {exc}")
        return response

    def handle_save_map(self, request, response):
        try:
            fused_path, raw_path = self._prepare_bag_paths(request.file_path)
            topic_base = request.topic_name or self.save_map_default_topic
            fused_topic = self._resolve_topic_name(topic_base)
            raw_topic = self._resolve_topic_name(f"{topic_base}_raw")

            fused_layer_names = self._collect_fused_layer_names()
            raw_layer_names = self._map.list_layers()
            self.get_logger().info(
                f"Saving map: fused layers={fused_layer_names}, raw layers={raw_layer_names}"
            )

            fused_layers = self._map.export_layers(fused_layer_names)
            raw_layers = self._map.export_layers(raw_layer_names)
            self.get_logger().info(
                f"Exported raw layer keys: {list(raw_layers.keys())}"
            )
            if "elevation" in fused_layers:
                n_finite = int(np.isfinite(fused_layers["elevation"]).sum())
                self.get_logger().info(f"save_map: fused 'elevation' finite cells={n_finite}")
            if "is_valid" in raw_layers:
                n_valid = int((raw_layers["is_valid"] > 0.5).sum())
                self.get_logger().info(f"save_map: raw 'is_valid' valid cells={n_valid}")

            gm_fused = self._build_grid_map_message(
                fused_layer_names,
                fused_layers,
                self._collect_basic_layers(),
            )
            gm_raw = self._build_grid_map_message(
                raw_layer_names,
                raw_layers,
                ['elevation'],
            )
            self.get_logger().info(
                f"Built fused msg layers={gm_fused.layers}, raw msg layers={gm_raw.layers}"
            )

            self._write_grid_map_bag(fused_path, fused_topic, gm_fused)
            self._write_grid_map_bag(raw_path, raw_topic, gm_raw)

            response.success = True
        except Exception as exc:
            self.get_logger().error(f"save_map failed: {exc}")
            response.success = False
        return response

    def handle_load_map(self, request, response):
        try:
            fused_path = Path(request.file_path).expanduser().resolve()
            raw_path = Path(f"{fused_path}_raw")
            if not fused_path.exists():
                raise FileNotFoundError(f"Fused map bag '{fused_path}' does not exist.")
            if not raw_path.exists():
                raise FileNotFoundError(f"Raw map bag '{raw_path}' does not exist.")

            topic_base = request.topic_name or self.save_map_default_topic
            fused_topic = self._resolve_topic_name(topic_base)
            raw_topic = self._resolve_topic_name(f"{topic_base}_raw")

            fused_msg = self._read_latest_grid_map(fused_path, fused_topic)
            raw_msg = self._read_latest_grid_map(raw_path, raw_topic)

            fused_layers, _ = self._grid_map_to_numpy(fused_msg)
            raw_layers, geometry = self._grid_map_to_numpy(raw_msg)

            self._map.set_full_map(fused_layers, raw_layers, geometry)

            pose_position = raw_msg.info.pose.position
            pose_orientation = raw_msg.info.pose.orientation
            self._map_t = Vector3(x=pose_position.x, y=pose_position.y, z=pose_position.z)
            self._map_q = Quaternion(
                x=pose_orientation.x,
                y=pose_orientation.y,
                z=pose_orientation.z,
                w=pose_orientation.w,
            )
            self._last_t = self.get_clock().now().to_msg()
            self._last_fused_t = self._last_t
            self._republish_all_once()
            # Quick sanity: the restored elevation should contain at least some finite values.
            tmp = np.zeros((self._map.cell_n - 2, self._map.cell_n - 2), dtype=np.float32)
            self._map.get_map_with_name_ref("elevation", tmp)
            n_finite = int(np.isfinite(tmp).sum())
            self.get_logger().info(f"load_map: restored 'elevation' finite cells={n_finite}")

            response.success = True
        except Exception as exc:
            self.get_logger().error(f"load_map failed: {exc}")
            response.success = False
        return response

    def _grid_map_to_numpy(self, grid_map_msg: GridMap):
        if len(grid_map_msg.layers) != len(grid_map_msg.data):
            raise ValueError("Mismatch between GridMap layers and data arrays.")

        arrays: Dict[str, np.ndarray] = {}
        for name, array_msg in zip(grid_map_msg.layers, grid_map_msg.data):
            arrays[name] = decode_multiarray_to_rows_cols(name, array_msg)

        center = np.array(
            [
                grid_map_msg.info.pose.position.x,
                grid_map_msg.info.pose.position.y,
                grid_map_msg.info.pose.position.z,
            ],
            dtype=np.float32,
        )
        orientation = np.array(
            [
                grid_map_msg.info.pose.orientation.x,
                grid_map_msg.info.pose.orientation.y,
                grid_map_msg.info.pose.orientation.z,
                grid_map_msg.info.pose.orientation.w,
            ],
            dtype=np.float32,
        )

        geometry = GridGeometry(
            length_x=grid_map_msg.info.length_x,
            length_y=grid_map_msg.info.length_y,
            resolution=grid_map_msg.info.resolution,
            center=center,
            orientation=orientation,
        )
        return arrays, geometry

    def _extract_layout_shape(self, array_msg: Float32MultiArray) -> tuple:
        if array_msg.layout.dim:
            cols = array_msg.layout.dim[0].size or 1
            rows = array_msg.layout.dim[1].size if len(array_msg.layout.dim) > 1 else (
                len(array_msg.data) // cols if cols else len(array_msg.data)
            )
        else:
            cols = int(math.sqrt(len(array_msg.data)))
            rows = cols
        return cols, rows

    def _collect_fused_layer_names(self) -> List[str]:
        fused: List[str] = []
        for config in self.my_publishers.values():
            fused.extend(config.get('layers', []))
        if not fused:
            fused = ['elevation']
        ordered: List[str] = []
        for name in fused:
            if name not in ordered:
                ordered.append(name)
        return ordered

    def _collect_basic_layers(self) -> List[str]:
        basics: List[str] = []
        for config in self.my_publishers.values():
            basics.extend(config.get('basic_layers', []))
        if not basics:
            basics = ['elevation']
        ordered: List[str] = []
        for name in basics:
            if name not in ordered:
                ordered.append(name)
        return ordered

    def _build_grid_map_message(
        self,
        layer_names: List[str],
        layer_data: Dict[str, np.ndarray],
        basic_layers: List[str],
    ) -> GridMap:
        gm = GridMap()
        gm.header.frame_id = self.map_frame
        gm.header.stamp = self._last_t if self._last_t is not None else self.get_clock().now().to_msg()
        gm.info.resolution = self._map.resolution
        actual_map_length = (self._map.cell_n - 2) * self._map.resolution
        gm.info.length_x = actual_map_length
        gm.info.length_y = actual_map_length

        center = self._get_map_center()
        gm.info.pose.position.x = float(center[0])
        gm.info.pose.position.y = float(center[1])
        gm.info.pose.position.z = float(center[2])
        if self._map_q is not None:
            gm.info.pose.orientation.x = self._map_q.x
            gm.info.pose.orientation.y = self._map_q.y
            gm.info.pose.orientation.z = self._map_q.z
            gm.info.pose.orientation.w = self._map_q.w
        else:
            gm.info.pose.orientation.w = 1.0

        gm.layers = []
        gm.basic_layers = basic_layers
        for name in layer_names:
            data = layer_data.get(name)
            if data is None:
                continue
            gm.layers.append(name)
            gm.data.append(self._numpy_to_multiarray(data))
        gm.outer_start_index = 0
        gm.inner_start_index = 0
        return gm

    def _numpy_to_multiarray(self, data: np.ndarray, layout: str = "gridmap_column") -> Float32MultiArray:
        return encode_layer_to_multiarray(data, layout=layout)

    def _resolve_service_name(self, suffix: str) -> str:
        base = self.service_namespace
        if not base:
            base = f"/{self.get_name()}"
        return f"{base}/{suffix}".replace('//', '/')

    def _resolve_topic_name(self, topic: str) -> str:
        topic = topic.strip('/') or self.save_map_default_topic
        base = self.service_namespace
        if not base:
            base = f"/{self.get_name()}"
        return f"{base}/{topic}".replace('//', '/')

    def _prepare_bag_paths(self, file_path: str):
        if not file_path:
            raise ValueError("file_path must be provided.")
        fused_path = Path(file_path).expanduser().resolve()
        raw_path = Path(f"{fused_path}_raw")
        if fused_path.exists():
            raise FileExistsError(f"Bag path '{fused_path}' already exists.")
        if raw_path.exists():
            raise FileExistsError(f"Bag path '{raw_path}' already exists.")
        fused_path.parent.mkdir(parents=True, exist_ok=True)
        return fused_path, raw_path

    def _make_topic_metadata(self, topic: str) -> rosbag2_py.TopicMetadata:
        msg_type = "grid_map_msgs/msg/GridMap"
        serialization_format = "cdr"
        return rosbag2_py.TopicMetadata(0, topic, msg_type, serialization_format)

    def _write_grid_map_bag(self, path: Path, topic: str, grid_map_msg: GridMap) -> None:
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id=self.save_map_storage_id)
        converter_options = rosbag2_py.ConverterOptions('', '')
        writer.open(storage_options, converter_options)
        topic_metadata = self._make_topic_metadata(topic)
        writer.create_topic(topic_metadata)
        writer.write(topic, serialize_message(grid_map_msg), self.get_clock().now().nanoseconds)

    def _read_latest_grid_map(self, path: Path, topic: str) -> GridMap:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id=self.save_map_storage_id)
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        latest = None
        while reader.has_next():
            current_topic, data, _ = reader.read_next()
            if current_topic != topic:
                continue
            msg = deserialize_message(data, GridMap)
            latest = msg
        if latest is None:
            raise ValueError(f"No messages for topic '{topic}' in bag '{path}'.")
        return latest

    def _get_map_center(self) -> np.ndarray:
        center = np.zeros((1, 3), dtype=np.float32)
        self._map.get_center_position(center)
        return center[0]

    def _republish_all_once(self) -> None:
        if self._map_q is None:
            return
        for key in self._publishers_dict.keys():
            self.publish_map(key)

    def _normalize_namespace(self, value: str) -> str:
        value = value.strip() if value else ''
        if not value:
            return ''
        if not value.startswith('/'):
            value = f'/{value}'
        return value.rstrip('/')

    def safe_lookup_transform(self, target_frame, source_frame, time):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time
            )
        except tf2_ros.ExtrapolationException:
            if not self.allow_latest_tf_fallback:
                self.get_logger().warning(
                    f"Stamped transform from '{source_frame}' to '{target_frame}' unavailable; dropping input",
                    throttle_duration_sec=5.0
                )
                return None
            # Time is in the future/past, try with latest available
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
            # NOTE: The second lookup can also throw ExtrapolationException (e.g., TF buffer not populated yet,
            # or timestamps are discontinuous during sim resets). If we don't catch it here the whole node dies.
            except (
                tf2.LookupException,
                tf2.ConnectivityException,
                tf2.ExtrapolationException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warning(
                    f"Transform from '{source_frame}' to '{target_frame}' not available: {e}",
                    throttle_duration_sec=5.0
                )
                return None
        except tf2.LookupException as e:
            # Frame doesn't exist
            self.get_logger().warning(
                f"Frame '{target_frame}' or '{source_frame}' does not exist: {e}",
                throttle_duration_sec=5.0
            )
            return None
        except tf2.ConnectivityException as e:
            # No transform path between frames
            self.get_logger().warning(
                f"No transform path from '{source_frame}' to '{target_frame}': {e}",
                throttle_duration_sec=5.0
            )
            return None
        except Exception as e:
            # Catch any other unexpected TF2 errors
            self.get_logger().warning(
                f"Unexpected TF2 error for transform from '{source_frame}' to '{target_frame}': {e}",
                throttle_duration_sec=5.0
            )
            return None

    def image_callback(self, camera_msg: Image, camera_info_msg: CameraInfo, sub_key: str) -> None:
        self._last_t = camera_msg.header.stamp

        frame_sensor_id = camera_msg.header.frame_id
        if not frame_sensor_id:
            raise ValueError("Image header.frame_id is empty.")

        semantic_img = self.cv_bridge.imgmsg_to_cv2(camera_msg, desired_encoding="passthrough")
        if len(semantic_img.shape) != 2:
            semantic_img = [semantic_img[:, :, idx] for idx in range(semantic_img.shape[2])]
        else:
            semantic_img = [semantic_img]

        K = np.array(camera_info_msg.k, dtype=np.float32).reshape(3, 3)
        D = np.array(camera_info_msg.d, dtype=np.float32).reshape(-1, 1)

        if frame_sensor_id == self.map_frame:
            t_np = np.zeros(3, dtype=np.float32)
            R = np.eye(3, dtype=np.float32)
        else:
            transform_camera_to_map = self.safe_lookup_transform(
                self.map_frame,
                frame_sensor_id,
                camera_msg.header.stamp,
            )
            if transform_camera_to_map is None:
                return
            t = transform_camera_to_map.transform.translation
            q = transform_camera_to_map.transform.rotation
            t_np = np.array([t.x, t.y, t.z], dtype=np.float32)
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)

        channels = self.resolve_image_channels(sub_key)
        if not channels:
            return

        self._map.input_image(
            semantic_img,
            channels,
            R,
            t_np,
            K,
            D,
            camera_info_msg.distortion_model,
            camera_info_msg.height,
            camera_info_msg.width,
        )
        self._image_process_counter += 1
        self._last_fused_t = camera_msg.header.stamp

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    @staticmethod
    def _stamp_text(stamp_ns: int) -> str:
        return f"{stamp_ns // 1000000000}.{stamp_ns % 1000000000:09d}"

    def pointcloud_callback(self, msg: PointCloud2, sub_key: str) -> None:
        self._input_stats['received'] += 1
        self._last_received_stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if self.stamped_tf_queue_enabled:
            self._enqueue_pointcloud(msg, sub_key)
            return

        # Inherited compatibility path. RUBI does not use this path, but legacy
        # configurations can keep immediate lookup and opt-in latest fallback.
        self._last_t = msg.header.stamp
        try:
            pts, channels = self._prepare_pointcloud(msg, sub_key)
        except (ValueError, TypeError) as exc:
            self._input_stats['invalid_drops'] += 1
            self.get_logger().warning(f"Invalid PointCloud2 input: {exc}")
            return
        if pts.size == 0:
            self._input_stats['invalid_drops'] += 1
            return

        frame_sensor_id = msg.header.frame_id
        if not frame_sensor_id:
            self._input_stats['invalid_drops'] += 1
            return

        if frame_sensor_id == self.map_frame:
            t_np = np.zeros(3, dtype=np.float32)
            R = np.eye(3, dtype=np.float32)
        else:
            transform_sensor_to_map = self.safe_lookup_transform(
                self.map_frame,
                frame_sensor_id,
                msg.header.stamp,
            )
            if transform_sensor_to_map is None:
                return
            t_np, R = self._transform_to_numpy(transform_sensor_to_map)

        self._map.input_pointcloud(pts, channels, R, t_np, 0, 0)
        self._pointcloud_process_counter += 1
        self._input_stats['fused'] += 1
        self._last_fused_t = msg.header.stamp

    def _prepare_pointcloud(self, msg: PointCloud2, sub_key: str):
        additional_channels = list(self.param.subscriber_cfg[sub_key].get("channels", []))
        channels = ["x", "y", "z"] + additional_channels

        if additional_channels:
            points = rnp.numpify(msg)
            if points is None:
                raise ValueError("ros2_numpy returned no points")

            if isinstance(points, dict):
                if not points:
                    raise ValueError("ros2_numpy returned an empty point dictionary")
                if "xyz" in points:
                    xyz_array = np.array(points["xyz"])
                    if xyz_array.ndim == 2 and xyz_array.shape[1] == 3:
                        pts = xyz_array
                    elif xyz_array.ndim == 1:
                        pts = xyz_array.reshape(-1, 3)
                    else:
                        pts = xyz_array[:, :3]
                elif all(name in points for name in ("x", "y", "z")):
                    pts = np.column_stack(
                        (
                            np.array(points["x"]).flatten(),
                            np.array(points["y"]).flatten(),
                            np.array(points["z"]).flatten(),
                        )
                    )
                else:
                    raise ValueError(
                        f"PointCloud2 dict for '{sub_key}' is missing xyz fields. "
                        f"Available: {list(points.keys())}"
                    )
                for channel in additional_channels:
                    if channel not in points:
                        raise ValueError(
                            f"PointCloud2 for '{sub_key}' is missing configured channel '{channel}'."
                        )
                    data = np.array(points[channel]).flatten()
                    if data.ndim == 1:
                        data = data[:, np.newaxis]
                    pts = np.hstack((pts, data))
            else:
                if points.size == 0:
                    raise ValueError("cloud is empty")
                pts = rnp.point_cloud2.get_xyz_points(points)
                for channel in additional_channels:
                    if not hasattr(points, "dtype") or channel not in points.dtype.names:
                        raise ValueError(
                            f"PointCloud2 for '{sub_key}' is missing configured channel '{channel}'."
                        )
                    data = points[channel].flatten()
                    if data.ndim == 1:
                        data = data[:, np.newaxis]
                    pts = np.hstack((pts, data))
            pts = np.asarray(pts)
            pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
        else:
            pts = _pointcloud2_xyz_f32(msg)
        return pts, channels

    def _enqueue_pointcloud(self, msg: PointCloud2, sub_key: str) -> None:
        if self._epoch_faulted or self._fusion_faulted:
            self._input_stats['invalid_drops'] += 1
            return
        frame_id = str(msg.header.frame_id).strip()
        payload_bytes = len(msg.data)
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if not frame_id or payload_bytes == 0:
            self._input_stats['invalid_drops'] += 1
            return

        item = PendingPointCloud(
            message=msg,
            subscriber_key=sub_key,
            stamp_ns=stamp_ns,
            received_monotonic=monotonic_time.monotonic(),
            payload_bytes=payload_bytes,
        )
        result, overflowed = self._pending_clouds.push(item)
        if result != 'accepted':
            if result in ('duplicate', 'out_of_order'):
                self._input_stats['duplicate_out_of_order_drops'] += 1
            elif result == 'oversize':
                self._input_stats['overflow_drops'] += 1
            else:
                self._input_stats['invalid_drops'] += 1
            self.get_logger().warning(
                f"Dropping PointCloud2 stamp={self._stamp_text(stamp_ns)}: {result}",
                throttle_duration_sec=5.0,
            )
            return

        if overflowed:
            self._input_stats['overflow_drops'] += len(overflowed)
            overflow_stamps = {d.stamp_ns for d in overflowed}
            if self._prepared_pending_stamp_ns in overflow_stamps:
                self._clear_prepared_pending()
            self.get_logger().warning(
                "Stamped-TF queue overflow; dropped oldest %d scan(s), pending=%d bytes=%d"
                % (len(overflowed), len(self._pending_clouds), self._pending_clouds.payload_bytes),
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _transform_to_numpy(transform):
        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)
        return trans, rot

    def _lookup_original_stamp(self, source_frame: str, stamp):
        if source_frame == self.map_frame:
            return True, None, None
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, source_frame, stamp
            )
            return True, transform, None
        except Exception as exc:
            return False, None, str(exc)

    def _clear_prepared_pending(self) -> None:
        self._prepared_pending_stamp_ns = None
        self._prepared_pending = None

    def _discard_pending_head(self) -> PendingPointCloud:
        item = self._pending_clouds.pop()
        if self._prepared_pending_stamp_ns == item.stamp_ns:
            self._clear_prepared_pending()
        return item

    def _observe_ros_clock(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_ros_now_ns is not None and now_ns < self._last_ros_now_ns:
            dropped = self._pending_clouds.clear()
            self._clear_prepared_pending()
            self._input_stats['reset_drops'] += dropped
            self._epoch_faulted = True
            self._last_fused_t = None
            try:
                self._tf_buffer.clear()
            except AttributeError:
                pass
            self.get_logger().fatal(
                "ROS time moved backwards (%d -> %d). Fusion and normal publication are stopped; "
                "restart/reset both elevation_mapping_node and rubi_global_heightmap_wrapper "
                "before accepting the new epoch." % (self._last_ros_now_ns, now_ns)
            )
            return False
        self._last_ros_now_ns = now_ns
        return True

    def process_pending_pointclouds(self) -> None:
        if not self.stamped_tf_queue_enabled:
            return
        if self._epoch_faulted or self._fusion_faulted or not self._observe_ros_clock():
            return

        processed = 0
        while processed < self.tf_max_scans_per_tick:
            item = self._pending_clouds.peek()
            if item is None:
                return
            now_monotonic = monotonic_time.monotonic()
            wait = now_monotonic - item.received_monotonic
            if wait >= self.tf_wait_timeout:
                self._discard_pending_head()
                self._input_stats['timeout_drops'] += 1
                errors = self._last_tf_errors if self._last_tf_error_stamp_ns == item.stamp_ns else {}
                sensor_error = errors.get('sensor', 'not checked')
                base_error = errors.get('base', 'not checked')
                self.get_logger().warning(
                    "Stamped TF timeout target='%s' sensor_source='%s' base_source='%s' "
                    "stamp=%s wait=%.3fs sensor_error=%s base_error=%s"
                    % (self.map_frame, item.message.header.frame_id, self.base_frame,
                       self._stamp_text(item.stamp_ns), wait, sensor_error, base_error),
                    throttle_duration_sec=5.0,
                )
                processed += 1
                continue

            if self._prepared_pending_stamp_ns != item.stamp_ns:
                try:
                    pts, channels = self._prepare_pointcloud(item.message, item.subscriber_key)
                    if pts.size == 0:
                        raise ValueError("cloud has no finite XYZ points")
                except Exception as exc:
                    self._discard_pending_head()
                    self._input_stats['invalid_drops'] += 1
                    self.get_logger().warning(
                        f"Invalid queued PointCloud2 stamp={self._stamp_text(item.stamp_ns)}: {exc}"
                    )
                    processed += 1
                    continue
                self._prepared_pending_stamp_ns = item.stamp_ns
                self._prepared_pending = (pts, channels)

            sensor_ready, sensor_tf, sensor_error = self._lookup_original_stamp(
                item.message.header.frame_id, item.message.header.stamp
            )
            base_ready, base_tf, base_error = self._lookup_original_stamp(
                self.base_frame, item.message.header.stamp
            )
            self._last_tf_errors = {'sensor': sensor_error, 'base': base_error}
            self._last_tf_error_stamp_ns = item.stamp_ns
            if not sensor_ready or not base_ready:
                # Return promptly so this SingleThreadedExecutor can receive the
                # delayed /tf or /tf_static data before the next steady tick.
                return

            pts, channels = self._prepared_pending
            try:
                if base_tf is None:
                    base_trans = np.zeros(3, dtype=np.float32)
                    base_rot = np.eye(3, dtype=np.float32)
                    base_t = Vector3(x=0.0, y=0.0, z=0.0)
                    base_q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                else:
                    base_trans, base_rot = self._transform_to_numpy(base_tf)
                    base_t = base_tf.transform.translation
                    base_q = base_tf.transform.rotation
                if sensor_tf is None:
                    sensor_trans = np.zeros(3, dtype=np.float32)
                    sensor_rot = np.eye(3, dtype=np.float32)
                else:
                    sensor_trans, sensor_rot = self._transform_to_numpy(sensor_tf)

                # These calls must remain ordered and use the two transforms
                # captured above for this exact sensor stamp.
                self._map.move_to(base_trans, base_rot)
                self._map.input_pointcloud(pts, channels, sensor_rot, sensor_trans, 0, 0)
            except Exception as exc:
                # move_to/input_pointcloud mutate GPU state without rollback.
                # Never publish that possibly partial state as a normal result.
                dropped = self._pending_clouds.clear()
                self._clear_prepared_pending()
                self._fusion_faulted = True
                self._last_fused_t = None
                self.get_logger().fatal(
                    "GPU fusion submission failed at stamp=%s; stopped fusion/publication "
                    "because rollback is unavailable (discarded pending=%d): %s"
                    % (self._stamp_text(item.stamp_ns), dropped, exc)
                )
                return

            self._discard_pending_head()
            self._map_t = base_t
            self._map_q = base_q
            self._last_t = item.message.header.stamp
            self._last_fused_t = item.message.header.stamp
            self._pointcloud_process_counter += 1
            self._input_stats['fused'] += 1
            self._queue_wait_samples.append(wait)
            if len(self._queue_wait_samples) > 2048:
                del self._queue_wait_samples[:-2048]
            self._last_tf_errors = {}
            self._last_tf_error_stamp_ns = None
            processed += 1

    def publish_input_statistics(self) -> None:
        samples = self._queue_wait_samples
        if samples:
            p50, p95 = np.percentile(np.asarray(samples), [50, 95])
            maximum = max(samples)
        else:
            p50 = p95 = maximum = 0.0
        received_ns = self._last_received_stamp_ns
        fused_ns = None if self._last_fused_t is None else self._stamp_to_ns(self._last_fused_t)
        self.get_logger().info(
            "input_stats received=%d fused=%d pending=%d pending_bytes=%d "
            "timeout_drops=%d overflow_drops=%d invalid_drops=%d "
            "duplicate_out_of_order_drops=%d reset_drops=%d "
            "queue_wait_p50=%.3fs p95=%.3fs max=%.3fs last_received_stamp=%s "
            "last_fused_stamp=%s epoch_faulted=%s fusion_faulted=%s"
            % (
                self._input_stats['received'], self._input_stats['fused'],
                len(self._pending_clouds), self._pending_clouds.payload_bytes,
                self._input_stats['timeout_drops'], self._input_stats['overflow_drops'],
                self._input_stats['invalid_drops'],
                self._input_stats['duplicate_out_of_order_drops'],
                self._input_stats['reset_drops'], p50, p95, maximum,
                'none' if received_ns is None else self._stamp_text(received_ns),
                'none' if fused_ns is None else self._stamp_text(fused_ns),
                self._epoch_faulted, self._fusion_faulted,
            )
        )

    def pose_update(self) -> None:
        if self.stamped_tf_queue_enabled:
            # Queue mode moves the rolling map immediately before fusion using
            # the base transform captured for that same scan stamp.
            return
        if self._last_t is None:
            return
        transform = self.safe_lookup_transform(
            self.map_frame,
            self.base_frame,
            self._last_t
        )
        if transform is None:
            # Transform not available, skip pose update
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)
        self._map.move_to(trans, rot)
        self._map_t = t
        self._map_q = q

    def update_variance(self) -> None:
        self._map.update_variance()

    def update_time(self) -> None:
        self._map.update_time()

    def destroy_node(self) -> None:
        super().destroy_node()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMappingNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        # launch_testing / signal handlers can already have shut down the context.
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
