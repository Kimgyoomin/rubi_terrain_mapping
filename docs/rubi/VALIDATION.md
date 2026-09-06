# Bootstrap validation — 2026-09-06

## Executed on the development Mac

- AppleClang 21, CMake Release, `RUBI_BUILD_ROS2=OFF`.
- Portable core: **37 checks passed**, CTest **1/1 passed**.
- Backend publisher/TF contract: **5 Python unittest tests passed**.
- Core coverage: asymmetric CuPy array layout, offsets/strides, world cell centers,
  5/15 cm detail preservation, rolling-window exit, revisit replacement, duplicate
  posterior idempotence, invalid history, bounds, malformed input, reset and exact
  save/load with failed-load atomicity.
- Backend tests execute the real extracted publisher/TF methods with small ROS/GPU
  boundary doubles. They cover snapped center, no extra z offset, processed-input
  timestamp and strict versus legacy TF fallback. They do not run CUDA or DDS.

The inherited GPU pytest command was attempted but the current Python environment
has no `pytest` module. This Mac also has no ROS2/CUDA runtime. Those upstream
tests have not passed here, and no GPU performance/accuracy result is claimed.

## Humble verification

`rubi-humble.yml` is configured to build the actual ROS adapter, run its core tests,
then execute `scripts/test_rubi_wrapper_ros.py` over DDS. The smoke test covers
asymmetric world coordinates, 15 cm preservation, persistence outside the local
window, rejection of half-cell misalignment, save/clear/load and original stamps.

At preparation time this workflow has not yet run. Its result must be checked on
the published development commit. The GPU backend itself is not launched by this
workflow. Integrated CuPy, FAST-LIO, Gazebo and Orin runs remain outstanding.
