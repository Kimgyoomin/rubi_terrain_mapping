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

## Stamped-TF queue portable regression (2026-09-08)

Run from the repository root:

```bash
python3 -m py_compile \
  elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_node.py \
  elevation_mapping_cupy/elevation_mapping_cupy/stamped_tf_queue.py \
  rubi_mapping_bringup/launch/rubi_mapping.launch.py
python3 scripts/test_stamped_tf_queue.py
python3 scripts/test_rubi_backend_contract.py
```

The queue test executes the real FIFO implementation and the real queued fusion
controller method with ROS/GPU boundary doubles. It covers deterministic count
and payload eviction, oversize/zero/duplicate/out-of-order rejection, exact-stamp
lookup, delayed sensor/base TF, timeout-before-processing, `move_to` before GPU
fusion, no success bookkeeping on an exception, disabled queued pose timer,
ROS-clock reversal fail-stop, and launch YAML/CLI precedence. It does not launch
ROS, deliver DDS TF, execute CuPy/CUDA, run Gazebo, or measure map quality.

The `RUBI Humble wrapper` workflow invokes both portable backend scripts after
building the wrapper. Integrated delayed-TF delivery with the CuPy node remains a
desktop Humble/CUDA validation because the workflow intentionally has no GPU
backend environment.

### 2026-09-08 직접 실행 결과 (현재 Humble desktop checkout)

- `test_stamped_tf_queue.py`: 13 tests passed.
- `test_rubi_backend_contract.py`: 5 tests passed.
- portable wrapper CMake/CTest: `PASS 37 checks`, 1/1 test passed.
- isolated Humble wrapper colcon build/test: 1 package and 1/1 test passed;
  after adding clock-reset coverage, `test_rubi_wrapper_ros.py`: 2 tests passed
  over DDS (persistence/services and ROS-time backward-jump publication stop).
- 요청된 `colcon build --symlink-install --packages-up-to
  rubi_mapping_bringup ... -DBUILD_TESTING=OFF`: 5 packages passed.
- 실제 CuPy node + moving/yaw synthetic PointCloud2에서 TF를 0.15초 늦게
  전달: 마지막 표본 `received=30`, `fused=29`, `pending=1`, p50/p95/max
  queue wait 약 0.158초, timeout/overflow/invalid 0. 원래 cloud stamp와 TF
  stamp는 유지했다.
- 같은 입력에서 TF 전달을 0.70초로 늦춤: 마지막 표본 `received=25`,
  `fused=0`, `pending=3`, `timeout_drops=22`, `last_fused_stamp=none`.
  timeout 로그에는 sensor/base source, 요청 stamp와 원본 extrapolation
  message가 포함됐다.

두 synthetic 실행은 격리된 ROS domain, wall ROS time에서 수행했으며 정상
종료했다. 실제 Gazebo `/clock` pause/reset, FAST-LIO 전체 chain, MID-360 raw
scan, RViz geometry/height quality, CUDA 비동기 오류 검출, RTX 5090/Orin 성능은
이 결과로 검증됐다고 보지 않는다.

The inherited 101-test pytest suite was also attempted from its documented test
directory with the current dependencies: 27 passed (including CUDA availability,
kernel compile/update, map shifting and service tests), 2 pre-existing config
sanity cases failed on `$(find-pkg-share ...)`, and 72 upstream parameterized
cases errored because their relative `../../../config/weights.dat` test path does
not exist from that working directory. These failures are outside this queue
change and are not reported as a passing inherited suite.
