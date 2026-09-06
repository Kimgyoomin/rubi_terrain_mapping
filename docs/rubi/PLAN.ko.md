# RUBI GPU local elevation → global height map

현재 기준: 2026-09-06. 사용자가 확정한 방향은 **elevation_mapping_gpu_ros2를
기반으로 수정하고, FastDEM처럼 global planner가 사용할 누적 height map을
만드는 wrapper를 개발하는 것**이다. 별도 raw-point CPU height estimator를
새로 만드는 방향은 채택하지 않는다.

## 범위와 구조

- ROS2 Humble, MID-360 한 대, 실제 localization은 FAST-LIO.
- 센서는 하단 거꾸로 장착, driver에서 LiDAR/IMU FLU 변환. 변환을 중복 적용하지 않는다.
- GT는 mapping 오차를 분리하는 비교 실험 입력으로 사용한다.
- 정적 실내, 평지·경사로·5/15cm 단차, 최대 50×50m, 5cm 격자로 시작한다.
- RTX 5090 PC에서 입력/품질 검증 후 Jetson Orin NX(16GB 추정)에서 부하 측정.

```text
MID-360 + FAST-LIO pose/deskewed cloud
              ↓
CuPy GPU local elevation 추정
              ↓ raw elevation + variance + 실제 격자 geometry
RUBI global wrapper
  유효 셀 갱신 / window 밖 지형 보존 / 저장·복원
              ↓
global height map → 기존 global planner
```

local posterior들을 매번 독립 측정으로 평균/Kalman fusion하지 않는다. 첫 버전은
품질 조건을 통과한 최신 추정값으로 교체한다. 서로 다른 방문의 나쁜 추정이 좋은
과거값을 덮을 수 있으므로 재방문 품질은 별도 평가한다. invalid 셀은 과거 추정값을
저장하되 XYZ에서는 제외하는 초기 정책이다. 구멍을 free 또는 확정 obstacle로
자동 해석하지 않는다.

## 이번 bootstrap

작성된 코드: CuPy backend의 격자 중심/stamp/TF fallback 수정, C++ global
wrapper와 ROS2 adapter, 저장/불러오기/reset, RUBI 통합 launch/config.
구현 상세와 현재 제약은 [wrapper 문서](../../rubi_global_heightmap_wrapper/README.md)에 있다.

로컬 portable 검사와 ROS 메시지 연결 검증은 [검증 기록](VALIDATION.md)에서
구분한다. 아직 실제 LiDAR/시뮬레이션 bag, CuPy GPU 실행, Orin 실행으로
지도 품질이나 실시간성을 입증하지 않았다.

## 다음 검증 순서

1. 사용 중인 Humble/CuPy 버전과 FAST-LIO 출력 frame/stamp/밀도를 기록한다.
   map→body와 cloud registration이 일치하는지 확인한다. 30cm voxel world cloud를
   5cm terrain 입력으로 선택하지 않는다.
2. 동일한 flat/5/15cm 입력으로 CuPy local 지도와 wrapper global 지도를 비교한다.
   wrapper가 단차를 추가로 흐리거나 위치를 바꾸지 않아야 한다.
3. local window 이상 이동하고 재방문한다. 이전 지형 유지, 경계 이중화,
   재방문 높이 변화, invalid 처리에 따른 coverage 감소를 측정한다.
4. 같은 bag으로 FastDEM과 비교한다. plateau Δh, flat RMSE/MAD,
   edge transition width, false step, coverage를 함께 기록한다.
5. 기존 planner의 `input_heightmap_topic`을 연결한다. map 정렬, 별도 occupancy
   costmap, snapshot freshness와 nearest-height 구멍 보완 동작을 검증한다.
6. JetPack/전력 모드를 확정하고 Orin에서 GPU update, wrapper, DDS, 전체 map
   export 및 planner snapshot 비용을 측정한다. 5090 결과는 Orin 성능 보장이 아니다.

첫 실험에서는 global map 기준을 고정한다. FAST-LIO 재localization/map 보정이
변하면 이미 융합한 지형은 자동 수정되지 않으므로 reset/remap한다. Loop closure
일관성, 별도 global Bayesian fusion, submap 구조는 후속 범위다.

실제 frame audit, point timing/deskew, 움직이는 다리 self-filter, 센서 noise와
variance cutoff 보정은 아직 필요한 입력 검증이다. GPU 계산만으로 잘못된
extrinsic이나 scan motion distortion을 해결할 수 있다고 가정하지 않는다.
