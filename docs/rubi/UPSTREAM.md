# Upstream provenance

- Repository: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- Imported commit: `e08063fe6937a768471fbf251fa30ed7eaae25b9`
- The original commit ancestry is preserved. `upstream` denotes the original
  project and `origin` denotes `Kimgyoomin/rubi_terrain_mapping` in the development checkout.
- Original README: [README.upstream.md](../../README.upstream.md).
- Original root MIT copyright/license remains in [LICENSE](../../LICENSE).
- Optional `plane_segmentation_ros2` remains pinned by the upstream submodule.
  It is not required for the initial raw-elevation wrapper path.

## RUBI changes to the existing backend

1. Publish actual cell-snapped XY map center rather than continuous robot XY.
   Keep neutral pose z/orientation because the elevation layer includes world z.
2. Track the last successfully processed input timestamp separately from the
   last received input. Failed TF lookup cannot refresh a published old map.
   This is still a map snapshot stamp, not a per-cell observation timestamp.
3. Add `allow_latest_tf_fallback`. The inherited core configuration retains true;
   RUBI sets false so a timestamp extrapolation does not register a moving cloud
   using a different pose time.

GPU fusion kernels, the sensor noise model, traversability model and Mahalanobis-
named threshold computation are not rewritten in this bootstrap. Raw-only ROS
publishing does not remove internal traversability/PyTorch computation. Profiling
and a controlled same-bag comparison are required before changing those costs.

The existing backend load service historically stamps a restored map with load
time. Use the wrapper's save/load path for persistent maps with preserved times.
Backend service edits are not a supported live source of wrapper updates in this
bootstrap: the wrapper's duplicate policy assumes increasing sensor snapshot time.

Inherited workflows target `ros2` and a project-specific GPU runner. The new
`rubi-humble.yml` checks the wrapper on a GitHub-hosted Humble container; it does
not publish a website/container or claim CUDA coverage.
