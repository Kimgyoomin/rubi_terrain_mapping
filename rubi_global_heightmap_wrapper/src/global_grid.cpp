#include "rubi_global_heightmap_wrapper/global_grid.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace rubi {
namespace {
constexpr std::size_t kMaxCells = 5000000;
void validate(const Geometry& g) {
  if (g.frame.empty() || !std::isfinite(g.resolution) || g.resolution <= 0 ||
      !std::isfinite(g.origin_x) || !std::isfinite(g.origin_y) ||
      g.cols == 0 || g.rows == 0 || g.cols > kMaxCells / g.rows ||
      std::abs(g.origin_x) > 100000 || std::abs(g.origin_y) > 100000 ||
      g.resolution * std::max(g.cols, g.rows) > 100000) {
    throw std::invalid_argument("Invalid or oversized grid geometry");
  }
}
bool close(double a, double b, double tolerance = 1e-6) {
  return std::abs(a - b) <= tolerance;
}
std::int64_t aligned_offset(double value) {
  const double nearest = std::round(value);
  if (!std::isfinite(value) || std::abs(value) > 1e9 ||
      std::abs(value - nearest) > 1e-3) {
    throw std::invalid_argument("Local/global cell boundaries are not aligned");
  }
  return static_cast<std::int64_t>(nearest);
}
}  // namespace

GlobalGrid::GlobalGrid(Geometry geometry, float max_variance)
    : geometry_(std::move(geometry)), max_variance_(max_variance) {
  validate(geometry_);
  if (!std::isfinite(max_variance_) || max_variance_ <= 0) {
    throw std::invalid_argument("max_variance must be finite and positive");
  }
  cells_.resize(geometry_.rows * geometry_.cols);
}

Update GlobalGrid::apply(const Patch& patch) {
  const auto& g = patch.geometry;
  validate(g);
  if (g.frame != geometry_.frame || !close(g.resolution, geometry_.resolution, 1e-9)) {
    throw std::invalid_argument("Patch frame/resolution differs from global grid");
  }
  if (patch.elevation.size() != g.rows * g.cols ||
      patch.variance.size() != patch.elevation.size()) {
    throw std::invalid_argument("Patch layer shape mismatch");
  }
  const auto x0 = aligned_offset((g.origin_x - geometry_.origin_x) / g.resolution);
  const auto y0 = aligned_offset((g.origin_y - geometry_.origin_y) / g.resolution);
  if (patch.stamp_ns <= 0 || patch.stamp_ns < last_snapshot_ns_) {
    throw std::invalid_argument("Nonpositive/backwards snapshot stamp; reset for a new run");
  }
  Update result;
  if (patch.stamp_ns == last_snapshot_ns_) {
    result.duplicate = true;
    return result;
  }
  for (std::size_t y = 0; y < g.rows; ++y) {
    for (std::size_t x = 0; x < g.cols; ++x) {
      const auto gx = x0 + static_cast<std::int64_t>(x);
      const auto gy = y0 + static_cast<std::int64_t>(y);
      if (gx < 0 || gy < 0 || gx >= static_cast<std::int64_t>(geometry_.cols) ||
          gy >= static_cast<std::int64_t>(geometry_.rows)) {
        ++result.outside;
        continue;
      }
      const auto local = y * g.cols + x;
      auto& cell = cells_[static_cast<std::size_t>(gy) * geometry_.cols +
                          static_cast<std::size_t>(gx)];
      const float height = patch.elevation[local];
      const float variance = patch.variance[local];
      if (!std::isfinite(height) || !std::isfinite(variance) || variance < 0 ||
          variance > max_variance_) {
        // Retain history, but do not advertise an invalid local estimate as usable.
        cell.usable = false;
        ++result.invalid;
        continue;
      }
      cell = {height, variance, patch.stamp_ns, true};
      ++result.accepted;
    }
  }
  last_snapshot_ns_ = patch.stamp_ns;
  return result;
}

void GlobalGrid::clear() {
  std::fill(cells_.begin(), cells_.end(), Cell{});
  last_snapshot_ns_ = 0;
}

std::vector<Point> GlobalGrid::surface() const {
  std::vector<Point> points;
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    const auto& cell = cells_[i];
    if (!cell.usable) continue;
    points.push_back({
      static_cast<float>(geometry_.origin_x + (i % geometry_.cols + 0.5) * geometry_.resolution),
      static_cast<float>(geometry_.origin_y + (i / geometry_.cols + 0.5) * geometry_.resolution),
      cell.elevation});
  }
  return points;
}

void GlobalGrid::save(std::ostream& out) const {
  std::size_t count = 0;
  for (const auto& cell : cells_) if (cell.estimate_snapshot_ns > 0) ++count;
  out << "RUBI_GLOBAL_HEIGHTMAP 1\n" << std::setprecision(17)
      << std::quoted(geometry_.frame) << ' ' << geometry_.resolution << ' '
      << geometry_.origin_x << ' ' << geometry_.origin_y << ' '
      << geometry_.cols << ' ' << geometry_.rows << ' '
      << max_variance_ << ' ' << last_snapshot_ns_ << ' ' << count << '\n';
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    const auto& c = cells_[i];
    if (c.estimate_snapshot_ns <= 0) continue;
    out << i << ' ' << c.elevation << ' ' << c.variance << ' '
        << c.estimate_snapshot_ns << ' ' << (c.usable ? 1 : 0) << '\n';
  }
  if (!out) throw std::runtime_error("Failed to write heightmap snapshot");
}

void GlobalGrid::load(std::istream& in) {
  std::string magic;
  int version = 0;
  Geometry incoming;
  float max_variance = 0;
  std::int64_t stamp = 0;
  std::size_t count = 0;
  if (!(in >> magic >> version) || magic != "RUBI_GLOBAL_HEIGHTMAP" || version != 1 ||
      !(in >> std::quoted(incoming.frame) >> incoming.resolution >> incoming.origin_x >>
        incoming.origin_y >> incoming.cols >> incoming.rows >> max_variance >> stamp >> count)) {
    throw std::invalid_argument("Malformed heightmap header");
  }
  validate(incoming);
  if (incoming.frame != geometry_.frame || incoming.rows != geometry_.rows ||
      incoming.cols != geometry_.cols || !close(incoming.resolution, geometry_.resolution, 1e-9) ||
      !close(incoming.origin_x, geometry_.origin_x) || !close(incoming.origin_y, geometry_.origin_y) ||
      !std::isfinite(max_variance) || !close(max_variance, max_variance_) ||
      stamp < 0 || count > cells_.size() || (count > 0 && stamp == 0)) {
    throw std::invalid_argument("Saved map configuration differs or header is invalid");
  }
  std::vector<Cell> next(cells_.size());
  for (std::size_t n = 0; n < count; ++n) {
    std::size_t i = 0;
    Cell c;
    int usable = 0;
    if (!(in >> i >> c.elevation >> c.variance >> c.estimate_snapshot_ns >> usable) ||
        i >= next.size() || next[i].estimate_snapshot_ns != 0 ||
        !std::isfinite(c.elevation) || !std::isfinite(c.variance) ||
        c.variance < 0 || c.variance > max_variance_ ||
        c.estimate_snapshot_ns <= 0 || c.estimate_snapshot_ns > stamp ||
        (usable != 0 && usable != 1)) {
      throw std::invalid_argument("Malformed/duplicate heightmap cell");
    }
    c.usable = usable == 1;
    next[i] = c;
  }
  in >> std::ws;
  if (!in.eof()) throw std::invalid_argument("Unexpected trailing heightmap data");
  cells_.swap(next);
  last_snapshot_ns_ = stamp;
}

void GlobalGrid::save_pcd(std::ostream& out) const {
  const auto points = surface();
  out << "# RUBI global surface; variance/state are in map.rghm\nVERSION .7\n"
      << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH " << points.size()
      << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n"
      << std::setprecision(9);
  for (const auto& p : points) out << p.x << ' ' << p.y << ' ' << p.z << '\n';
  if (!out) throw std::runtime_error("Failed to write PCD");
}

std::vector<float> decode_cupy_layer(
    const std::vector<float>& data, std::size_t offset,
    const std::string& label0, std::size_t size0, std::size_t stride0,
    const std::string& label1, std::size_t size1, std::size_t stride1,
    std::size_t rows, std::size_t cols) {
  if (!rows || !cols || cols > kMaxCells / rows) throw std::invalid_argument("Invalid layer shape");
  const auto count = rows * cols;
  const bool column_major = label0 == "column_index" && label1 == "row_index";
  const bool row_major = label0 == "row_index" && label1 == "column_index";
  if ((!column_major && !row_major) ||
      size0 != (column_major ? cols : rows) || size1 != (column_major ? rows : cols) ||
      stride0 != count || stride1 != size1 || offset > data.size() ||
      data.size() - offset != count) {
    throw std::invalid_argument("Unsupported or inconsistent CuPy MultiArray layout");
  }
  std::vector<float> result(count);
  for (std::size_t y = 0; y < rows; ++y) {
    for (std::size_t x = 0; x < cols; ++x) {
      result[y * cols + x] = data[offset + (column_major ? x * rows + y : y * cols + x)];
    }
  }
  return result;
}
}  // namespace rubi
