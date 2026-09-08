# RUBI stamped-TF 큐 실행·검증 가이드

## 동작 계약

RUBI 설정은 `/livox/lidar_PointCloud2`의 원본 메시지, subscriber key,
원래 header stamp, monotonic 수신 시각을 FIFO에 보관한다. 콜백은 TF를
기다리지 않고 즉시 반환한다. steady-clock timer가 FIFO의 첫 메시지만 검사하며,
원래 stamp의 `map <- sensor`와 `map <- base_link`가 모두 준비되는 즉시 다음
순서로 한 번 처리한다.

```text
finite XYZ 준비
→ 동일 stamp의 sensor/base TF 두 개 확보
→ 기존 ElevationMap.move_to(base TF)
→ 기존 ElevationMap.input_pointcloud(sensor TF)
→ 성공 통계와 snapshot stamp 갱신
```

후행 scan을 먼저 융합하지 않는다. 기본 체류 예산은 0.5초이고 고정 지연이
아니다. count 20개 또는 보관 중인 PointCloud2 `data` 합계 64 MiB를 넘으면
가장 오래된 scan부터 폐기한다. 이는 프로세스 RSS나 DDS reader history 제한이
아니다. stamp 0(동일 API에서 latest로 해석될 수 있음), 중복/역순 stamp, 빈
frame/data, 유효 finite XYZ가 없는 cloud, 단일 64 MiB 초과 메시지는 융합하지
않는다.

queued 모드는 XYZ-only PointCloud2 한 개만 지원한다. image/semantic/multiple
sensor 설정은 시작 시 거부한다. upstream 호환 경로는 core YAML에서 queue가
꺼져 있으며, RUBI YAML만 strict queue를 켠다. queue와 latest fallback을 동시에
켜면 시작이 실패한다.

`fused`는 `input_pointcloud()` 호출이 Python 예외 없이 반환됐다는 뜻이다.
추가 CUDA synchronize를 넣지 않았으므로 전체 비동기 GPU 작업 완료나 지도
품질의 증거가 아니다. queue wait 통계는 monotonic 수신부터 처리 시작까지이며
FAST-LIO 계산 시간, sensor age, GPU 처리 시간과 다르다.

Gazebo pause 중에도 monotonic timeout은 진행한다. node의 ROS clock 자체가
역행한 경우에만 epoch reset으로 판정한다. 지연 도착한 과거 sensor stamp는
out-of-order drop일 뿐 reset 판정 근거가 아니다. clock 역행 또는 rollback 없는
GPU fusion 예외가 발생하면 local 정상 publication과 새 fusion을 fail-stop한다.
local/global mapper를 함께 재시작하기 전에는 자동 복구하지 않는다.

## 빌드와 실행

Gazebo의 기존 safe-start 순서를 지키고, 모든 관련 terminal에서 같은
`ROS_DOMAIN_ID=45`를 사용한다. `/clock`을 소비해야 하는 Gazebo, FAST-LIO,
mapping node는 모두 `use_sim_time=true`인지 확인한다.

```bash
cd ~/rubi_mapping_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to rubi_mapping_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

기존 명령으로 simulator와 localization을 각각 실행한다.

```bash
export ROS_DOMAIN_ID=45
ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py
```

```bash
export ROS_DOMAIN_ID=45
ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  use_sim_time:=true \
  publish_2d_map:=false \
  use_rviz:=true
```

mapping 실행:

```bash
source /opt/ros/humble/setup.bash
source ~/rubi_mapping_ws/install/setup.bash

export PYTHONPATH="$HOME/rubi_mapping_ws/.python_deps:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=45

ros2 launch rubi_mapping_bringup rubi_mapping.launch.py \
  use_sim_time:=true \
  cloud_topic:=/livox/lidar_PointCloud2 \
  base_frame:=base_link
```

`cloud_topic`과 `base_frame`을 생략하면 선택한 `backend_config` YAML의 값이
그대로 적용된다. 비어 있지 않은 CLI 값만 YAML을 override한다.

## 실행 전후 확인

```bash
ros2 param get /elevation_mapping_node use_sim_time
ros2 param get /elevation_mapping_node stamped_tf_queue_enabled
ros2 param get /elevation_mapping_node allow_latest_tf_fallback
ros2 param get /elevation_mapping_node base_frame
ros2 topic info /livox/lidar_PointCloud2 -v
ros2 topic info /tf -v
ros2 topic hz /livox/lidar_PointCloud2
ros2 topic hz /Odometry
```

전체 chain에서 scan stamp 조회가 되는지 확인한다. latest 조회만 반복하는 것은
검증이 아니다. 문제가 있으면 `map → camera_init → body → BODY → base_link →
livox_frame` 중 어느 edge가 요청 시각을 아직 포함하지 않는지 확인한다.

node 로그의 `input_stats`에서 다음을 기록한다.

- `received`, `fused`, `pending`, `pending_bytes`
- `timeout_drops`, `overflow_drops`, `invalid_drops`,
  `duplicate_out_of_order_drops`
- `queue_wait_p50`, `p95`, `max`
- `last_received_stamp`, `last_fused_stamp`
- `epoch_faulted`, `fusion_faulted`

snapshot stamp와 finite global cell 수는 다음처럼 독립적으로 확인한다.

```bash
ros2 topic echo /elevation_mapping_node/elevation_map_raw --once --field header.stamp
ros2 topic echo /rubi/global_elevation/cloud --once --field header.stamp
ros2 topic echo /rubi/global_elevation/cloud --once --field width
```

출력 Hz만으로 새 관측 융합을 판정하지 않는다. 새 fusion이 없으면 과거 snapshot
재발행 stamp도 그대로여야 한다. `width`는 wrapper가 XYZ로 내보낸 usable/finite
cell 수이며 전체 local elevation cell의 관측 시각을 뜻하지 않는다.

품질 확인은 정지 평지/고정 단차 → 느린 직진 → 느린 회전 순서로 수행한다.
각 단계에서 fused와 실제 snapshot stamp가 전진하는지, timeout/overflow가
누적되는지, usable cell 수와 RViz 경계/표면이 함께 타당한지 기록한다. 초기화
성공이나 topic Hz만으로 품질 완료를 선언하지 않는다.

실제 sensor 대신 moving/yaw 합성 장면에서 TF 전달 지연만 재현하려면 별도
terminal에서 다음을 실행할 수 있다. `0.15`는 허용 예이고 `0.70`은 기본
0.5초 timeout 폐기를 확인하는 예다. 합성 cloud/TF의 header stamp는 같고 TF의
DDS 전달만 늦어진다.

```bash
ros2 run elevation_mapping_cupy synthetic_pointcloud_tf_publisher.py --ros-args \
  -p pointcloud_topic:=/livox/lidar_PointCloud2 \
  -p map_frame:=map \
  -p base_frame:=base_link \
  -p sensor_frame:=livox_frame \
  -p tf_delivery_delay_s:=0.15 \
  -p publish_rate_hz:=5.0
```

이는 실제 FAST-LIO chain, raw MID-360 timestamp 의미, Gazebo `/clock`, 지도
정확도를 대체하는 시험이 아니다.

## reset과 rollback

latest fallback으로 만든 지도, ROS clock reset 전 지도, fusion fault 이후 지도는
재사용하지 않는다. mapping launch를 종료하여 local mapper와 wrapper를 함께
재시작한다. `global_heightmap.yaml`의 `load_path`는 빈 문자열로 유지하여 오염된
저장 map을 자동/수동 load하지 않는다. wrapper만 clear하고 이전 local map을
계속 publish하는 방식은 공동 reset이 아니다. planner cache도 별도로 재시작한다.

배포 전에 변경 patch를 보관하면 로컬 rollback이 가능하다.

```bash
cd ~/rubi_mapping_ws/src/rubi_terrain_mapping
git status --short --branch
git diff --binary > ~/rubi-stamped-tf-queue.patch
```

해당 patch만 되돌릴 때는 추가 사용자 변경이 없는 clean 작업 복사본에서 다음을
사용한다. 이후 다시 빌드한다.

```bash
git apply --check -R ~/rubi-stamped-tf-queue.patch
git apply -R ~/rubi-stamped-tf-queue.patch
```

변경을 commit한 뒤라면 공유 브랜치의 역사를 재작성하지 말고 해당 commit을
`git revert <commit>`으로 되돌린다. `reset --hard`, `git clean`, force push,
main 병합은 이 절차에 포함하지 않는다.

## 남은 한계

이 큐는 point-wise deskew가 아니다. raw scan header stamp가 scan 시작/끝/기준
시각 중 무엇인지 별도 확인해야 한다. `map → camera_init` 보정 변화, extrinsic,
다리 self-points, 센서 잡음, 2.5D 표현 한계는 이 패치로 해결되지 않는다. 이미
누적한 global terrain은 localization 보정에 맞추어 자동 재정렬되지 않는다.
