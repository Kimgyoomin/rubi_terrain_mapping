# Global heightmap wrapper

This node persists the pinned CuPy backend's local elevation estimates. It is
not a second height estimator or a raw LiDAR mapper.

## Input contract

- `grid_map_msgs/msg/GridMap` on `/elevation_mapping_node/elevation_map_raw`.
- Exactly one `elevation` and one `variance` layer; finite elevation represents
  a locally valid estimate. Variance must be finite, nonnegative and below the
  configured cutoff. No smoothing/inpainting layer is consumed.
- **CuPy native axes:** matrix row increases with world Y; column increases with
  world X. This is explicitly specific to this backend's export, not a promise
  to decode every producer's GridMap world-axis convention.
- Two labeled MultiArray dimensions, validated size/stride/data offset. Both
  CuPy column-major and row-major serialization are decoded. Circular start
  indices must be zero, as in this backend's publisher; others are rejected.
- Identity map orientation, pose.z=0, absolute elevation z. Published center must
  be the backend's actual snapped grid center. No extra robot pose transform.
- Resolution must equal the global grid. Cell boundary phase tolerance is
  0.001 cell (0.05 mm at 5 cm). Misaligned patches are rejected, not interpolated.
- Frame must match `map_frame`. Positive, increasing snapshot stamps; duplicates
  have no effect and backwards timestamps require an explicit experiment reset.

The subscriber is best effort with depth 2, compatible with the backend's
reliable publisher. A dropped local snapshot can lose terrain that exits the
window before another snapshot arrives; bag tests must check this coverage.

## Persistence rules

Accepted values replace the corresponding global cell's height and variance.
The variance is copied, not reduced by treating overlapping posteriors as
independent measurements. Cells outside the local window retain their state.

An invalid local cell marks the corresponding global cell unusable for XYZ,
while retaining its last accepted height/variance in the saved state. A later
accepted estimate restores it. This conservative first policy can reduce usable
coverage on re-entry; it must be measured rather than hidden by filling holes.

`estimate_snapshot_ns` means the snapshot carrying the estimate, not a measured
per-cell acquisition time. All scans within a local posterior are correlated.
This first wrapper does not estimate global pose covariance, correct an already
accumulated map after relocalization, or solve dynamic terrain removal.

## Output and services

`/rubi/global_elevation/cloud`: reliable/volatile depth 1, full packed FLOAT32
XYZ snapshot, height=1, one point at each usable global cell center, 1 Hz by
default. Frame is `map`; stamp is the last accepted local snapshot's stamp.
The node does not freshen old output with wall time. Export/packing is cached
between updates and skipped without subscribers.

```bash
ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
ros2 service call /rubi/global_elevation/load_map std_srvs/srv/Trigger '{}'
ros2 service call /rubi/global_elevation/clear_map std_srvs/srv/Trigger '{}'
```

Save creates a new directory below `output_directory` containing `map.rghm`
(geometry, cutoff, timestamps, retained estimates and usable state) and
`surface.pcd` (usable XYZ only). Load reads the startup parameter `load_path`
and requires matching configuration; a malformed load leaves the current map
unchanged. Saved times are preserved. Save/load and input callbacks are serialized,
so stop bag playback during file operations. A failed save can leave a partial
new directory; the service reports failure and never overwrites an old run.

All configuration parameters are read-only after startup; use a YAML override
and restart to change `load_path`, geometry, cutoff or topics.

**Reset consumers between experiments.** Clear affects only the wrapper. CuPy
must also be cleared/restarted, or its next posterior repopulates the old terrain.
The current planner rejects empty clouds, so this node suppresses empty output
and cannot clear that planner's cache. Its normal freshness checks still apply.
Restart the planner before evaluating a new run. Omitting unusable cells does not
make them explicit hazards because the existing planner can query nearby heights.

The default fixed grid has 1 million cells. Storage is bounded, but full XYZ
output and planner snapshot reconstruction still scale with observed coverage.
Orin performance has not been measured.
