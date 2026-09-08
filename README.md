# RUBI terrain mapping

ROS2 Humble terrain mapping for RUBI: **reuse CuPy GPU local elevation estimates,
then retain them in a persistent global height map for the existing planner.**

```text
MID-360 corrected raw PointCloud2 + FAST-LIO localization TF
                 ↓
elevation_mapping_cupy  (GPU height estimation)
                 ↓ raw elevation + variance, map-aligned local grid
rubi_global_heightmap_wrapper  (global persistence)
                 ↓
/rubi/global_elevation/cloud → global planner
```

The initial target is static indoor terrain, flat ground, ramps and 5/15 cm steps,
up to 50×50 m at 5 cm resolution. Target hardware is Jetson Orin NX, likely 16 GB;
the development simulator uses ROS2 Humble and an RTX 5090. Hardware performance
and real terrain accuracy have not been validated by the tests in this repository.

## Packages

| Package | Role |
|---|---|
| `elevation_mapping_cupy` | Inherited GPU estimator with small geometry/timestamp integration changes |
| `elevation_map_msgs` | Inherited backend interfaces |
| `rubi_global_heightmap_wrapper` | Fixed global grid, validated overwrite, history, XYZ export, save/load/reset |
| `rubi_mapping_bringup` | One MID-360 input, FAST-LIO frame defaults, integrated launch |

The wrapper does not implement a replacement raw-point height filter. It stores
accepted local posterior estimates without repeatedly averaging overlapping maps.
See [design and current status](docs/rubi/PLAN.ko.md),
[input/output contract](rubi_global_heightmap_wrapper/README.md), and
[upstream provenance and modifications](docs/rubi/UPSTREAM.md).

## Quick checks without ROS or CUDA

```bash
cmake -S rubi_global_heightmap_wrapper -B build/rubi-wrapper \
  -DRUBI_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/rubi-wrapper -j2
ctest --test-dir build/rubi-wrapper --output-on-failure
python3 scripts/test_rubi_backend_contract.py
python3 scripts/test_stamped_tf_queue.py
```

These test grid persistence and integration contracts, not GPU execution or LiDAR
accuracy. The `RUBI Humble wrapper` workflow additionally compiles the ROS adapter
and runs a synthetic DDS test without CUDA. See [validation](docs/rubi/VALIDATION.md).

## Run in an existing Humble/CuPy environment

Use the Ubuntu environment where the GPU backend dependencies already work.
The inherited Docker defaults and Python/CUDA dependencies have **not** been
validated for a new Orin installation. [Original setup documentation](README.upstream.md)
is retained as upstream reference, not a tested RUBI installation recipe.

Place this repository under the ROS workspace's `src/`, then from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  elevation_map_msgs elevation_mapping_cupy \
  rubi_global_heightmap_wrapper rubi_mapping_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch rubi_mapping_bringup rubi_mapping.launch.py use_sim_time:=true
```

This starts the GPU mapper and wrapper. Run the existing simulator and FAST-LIO
separately. The RUBI YAML default is `/livox/lidar_PointCloud2`, with `map` as the
fixed frame and `base_link` as the rolling-map frame. It uses a bounded,
non-blocking queue until both `map <- livox_frame` and `map <- base_link` are
available at the original scan stamp; latest-TF fallback is disabled. Empty
`cloud_topic`/`base_frame` launch arguments preserve the selected backend YAML,
while explicit non-empty CLI values override it.

An alternate input can be selected explicitly only after verifying its frame and
timestamp contract:

```bash
ros2 launch rubi_mapping_bringup rubi_mapping.launch.py \
  use_sim_time:=true cloud_topic:=/some/verified_cloud base_frame:=base_link
```

This does not create a GT bridge or apply another sensor-axis flip. The queue is
not point-wise deskew: moving raw scans still need their header timestamp meaning
and motion distortion validated. GPU noise parameters, body/leg filtering and
the variance cutoff still require controlled RUBI bag tests. See the
[stamped-TF runtime guide](docs/rubi/STAMPED_TF_QUEUE.ko.md).

For the existing planner, set
`input_heightmap_topic: /rubi/global_elevation/cloud` in its full configuration.
The compatibility XYZ cloud does not encode hazards or per-cell freshness; map
holes alone are not a validated collision-avoidance mechanism.

## Upstream and license

Based on [iit-DLSLab/elevation_mapping_gpu_ros2](https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2)
at `e08063fe6937a768471fbf251fa30ed7eaae25b9`, with Git history preserved.
The inherited [MIT license](LICENSE), notices, optional submodule and original
documentation are retained. Dependencies retain their respective licenses.
RUBI additions in this repository use MIT as stated in their package manifests.
