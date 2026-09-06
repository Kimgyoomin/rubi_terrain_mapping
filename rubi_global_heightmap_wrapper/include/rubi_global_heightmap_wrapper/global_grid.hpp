#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

namespace rubi {
struct Geometry {
  std::string frame = "map";
  double resolution = 0.05;
  double origin_x = -25.0;  // Lower cell boundary, not first cell center.
  double origin_y = -25.0;
  std::size_t cols = 1000;
  std::size_t rows = 1000;
};

struct Patch {
  Geometry geometry;
  std::int64_t stamp_ns = 0;
  // CuPy native convention: row increases with world Y, column with world X.
  std::vector<float> elevation;
  std::vector<float> variance;
};

struct Cell {
  float elevation = std::numeric_limits<float>::quiet_NaN();
  float variance = std::numeric_limits<float>::quiet_NaN();
  // Time of the snapshot carrying this estimate; not per-cell acquisition time.
  std::int64_t estimate_snapshot_ns = 0;
  bool usable = false;
};

struct Point { float x, y, z; };
struct Update {
  std::size_t accepted = 0;
  std::size_t invalid = 0;
  std::size_t outside = 0;
  bool duplicate = false;
};

class GlobalGrid {
 public:
  explicit GlobalGrid(Geometry geometry, float max_variance = 0.04F);
  Update apply(const Patch& patch);
  void clear();
  std::vector<Point> surface() const;
  void save(std::ostream& stream) const;
  void load(std::istream& stream);  // Validate completely before replacing state.
  void save_pcd(std::ostream& stream) const;
  const Geometry& geometry() const { return geometry_; }
  const std::vector<Cell>& cells() const { return cells_; }
  std::int64_t last_snapshot_ns() const { return last_snapshot_ns_; }

 private:
  Geometry geometry_;
  float max_variance_;
  std::vector<Cell> cells_;
  std::int64_t last_snapshot_ns_ = 0;
};

// Decode the pinned CuPy publisher's MultiArray, not arbitrary GridMap axes.
// Nonzero circular start indices must be rejected by the ROS adapter.
std::vector<float> decode_cupy_layer(
    const std::vector<float>& data, std::size_t offset,
    const std::string& label0, std::size_t size0, std::size_t stride0,
    const std::string& label1, std::size_t size1, std::size_t stride1,
    std::size_t rows, std::size_t cols);
}  // namespace rubi
