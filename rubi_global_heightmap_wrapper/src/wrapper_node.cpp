#include "rubi_global_heightmap_wrapper/global_grid.hpp"

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace rubi {
namespace {
std::size_t dimension(double length, double resolution) {
  const double cells = length / resolution;
  if (!std::isfinite(cells) || cells < 1 || cells > 5000000 ||
      std::abs(cells - std::round(cells)) > 1e-3) {
    throw std::invalid_argument("Grid length is not a positive integer number of cells");
  }
  return static_cast<std::size_t>(std::llround(cells));
}

Patch decode(const grid_map_msgs::msg::GridMap& msg) {
  if (msg.outer_start_index != 0 || msg.inner_start_index != 0) {
    throw std::invalid_argument("Expected unwrapped CuPy snapshot (zero start indices)");
  }
  const auto& q = msg.info.pose.orientation;
  const auto& t = msg.info.pose.position;
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 ||
      std::abs(q.z) > 1e-6 || std::abs(std::abs(q.w) - 1.0) > 1e-6 ||
      !std::isfinite(t.z) || std::abs(t.z) > 1e-6) {
    throw std::invalid_argument("Expected map-aligned CuPy elevation with neutral pose z/orientation");
  }
  Patch patch;
  auto& g = patch.geometry;
  g.frame = msg.header.frame_id;
  g.resolution = msg.info.resolution;
  if (!std::isfinite(g.resolution) || g.resolution <= 0) {
    throw std::invalid_argument("Invalid local resolution");
  }
  g.cols = dimension(msg.info.length_x, g.resolution);
  g.rows = dimension(msg.info.length_y, g.resolution);
  g.origin_x = t.x - msg.info.length_x / 2.0;
  g.origin_y = t.y - msg.info.length_y / 2.0;
  if (msg.header.stamp.nanosec >= 1000000000U) throw std::invalid_argument("Invalid timestamp");
  patch.stamp_ns = static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000LL +
                   msg.header.stamp.nanosec;
  if (msg.layers.size() != msg.data.size()) throw std::invalid_argument("Layer/data count mismatch");
  auto layer = [&](const std::string& name) {
    const auto found = std::find(msg.layers.begin(), msg.layers.end(), name);
    if (found == msg.layers.end() || std::count(msg.layers.begin(), msg.layers.end(), name) != 1) {
      throw std::invalid_argument("Missing or duplicate layer: " + name);
    }
    const auto& a = msg.data[static_cast<std::size_t>(found - msg.layers.begin())];
    if (a.layout.dim.size() != 2) throw std::invalid_argument("Expected two MultiArray dimensions");
    const auto& d0 = a.layout.dim[0];
    const auto& d1 = a.layout.dim[1];
    return decode_cupy_layer(a.data, a.layout.data_offset,
                            d0.label, d0.size, d0.stride, d1.label, d1.size, d1.stride,
                            g.rows, g.cols);
  };
  patch.elevation = layer("elevation");
  patch.variance = layer("variance");
  return patch;
}
}  // namespace

class WrapperNode : public rclcpp::Node {
 public:
  WrapperNode() : Node("rubi_global_heightmap_wrapper") {
    Geometry geometry;
    geometry.frame = setting<std::string>("map_frame", "map");
    geometry.resolution = setting<double>("resolution", 0.05);
    geometry.origin_x = setting<double>("origin_x", -25.0);
    geometry.origin_y = setting<double>("origin_y", -25.0);
    geometry.cols = dimension(setting<double>("length_x", 50.0), geometry.resolution);
    geometry.rows = dimension(setting<double>("length_y", 50.0), geometry.resolution);
    grid_ = std::make_unique<GlobalGrid>(geometry,
      static_cast<float>(setting<double>("max_variance", 0.04)));
    const auto convention = setting<std::string>("input_convention", "cupy_rows_y_cols_x");
    if (convention != "cupy_rows_y_cols_x") throw std::invalid_argument("Unsupported input convention");
    const auto input = setting<std::string>("input_topic", "/elevation_mapping_node/elevation_map_raw");
    const auto output = setting<std::string>("output_topic", "/rubi/global_elevation/cloud");
    output_directory_ = setting<std::string>("output_directory", "/tmp/rubi_mapping_results");
    load_path_ = setting<std::string>("load_path", "");
    const auto fps = setting<double>("publish_fps", 1.0);
    if (!std::isfinite(fps) || fps <= 0 || fps > 20) throw std::invalid_argument("Invalid publish_fps");
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::QoS(1).reliable());
    sub_ = create_subscription<grid_map_msgs::msg::GridMap>(input, rclcpp::QoS(2).best_effort(),
      [this](grid_map_msgs::msg::GridMap::ConstSharedPtr msg) {
        try {
          const auto result = grid_->apply(decode(*msg));
          if (!result.duplicate) dirty_ = true;
          if (result.outside > 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "%zu local cells outside fixed global bounds", result.outside);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected local map: %s", e.what());
        }
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() { publish(); });
    save_ = create_service<std_srvs::srv::Trigger>("/rubi/global_elevation/save_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          const auto tick = std::chrono::system_clock::now().time_since_epoch().count();
          const auto dir = std::filesystem::path(output_directory_) / ("run_" + std::to_string(tick));
          std::filesystem::create_directories(output_directory_);
          if (!std::filesystem::create_directory(dir)) throw std::runtime_error("Output already exists");
          std::ofstream map_file(dir / "map.rghm");
          grid_->save(map_file);
          map_file.close();
          if (!map_file) throw std::runtime_error("Snapshot flush failed");
          std::ofstream pcd_file(dir / "surface.pcd");
          grid_->save_pcd(pcd_file);
          pcd_file.close();
          if (!pcd_file) throw std::runtime_error("PCD flush failed");
          response->success = true;
          response->message = dir.string();
        } catch (const std::exception& e) { response->message = e.what(); }
      });
    load_ = create_service<std_srvs::srv::Trigger>("/rubi/global_elevation/load_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          if (load_path_.empty()) throw std::runtime_error("Set load_path at startup first");
          std::ifstream map_file(load_path_);
          if (!map_file) throw std::runtime_error("Cannot open load_path");
          grid_->load(map_file);
          dirty_ = true;
          response->success = true;
          response->message = "Loaded; original snapshot time preserved. Reset consumers between runs.";
        } catch (const std::exception& e) { response->message = e.what(); }
      });
    clear_ = create_service<std_srvs::srv::Trigger>("/rubi/global_elevation/clear_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        grid_->clear();
        cached_ = sensor_msgs::msg::PointCloud2{};
        dirty_ = true;
        response->success = true;
        response->message = "Cleared wrapper only. Clear/restart CuPy and planner for a new experiment.";
      });
    RCLCPP_INFO(get_logger(), "CuPy persistence wrapper: %zu x %zu cells, %.3f m, frame %s",
      geometry.cols, geometry.rows, geometry.resolution, geometry.frame.c_str());
  }

 private:
  template <typename T> T setting(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  void publish() {
    if (pub_->get_subscription_count() == 0 || grid_->last_snapshot_ns() <= 0) return;
    if (dirty_) {
      cached_ = sensor_msgs::msg::PointCloud2{};
      cached_.header.frame_id = grid_->geometry().frame;
      cached_.header.stamp = rclcpp::Time(grid_->last_snapshot_ns());
      cached_.height = 1;
      sensor_msgs::PointCloud2Modifier modifier(cached_);
      modifier.setPointCloud2Fields(3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);
      const auto points = grid_->surface();
      modifier.resize(points.size());
      if (!points.empty()) {
        sensor_msgs::PointCloud2Iterator<float> x(cached_, "x"), y(cached_, "y"), z(cached_, "z");
        for (const auto& p : points) { *x = p.x; *y = p.y; *z = p.z; ++x; ++y; ++z; }
      }
      cached_.is_dense = true;
      const std::uint16_t endian = 1;
      cached_.is_bigendian = *reinterpret_cast<const std::uint8_t*>(&endian) == 0;
      dirty_ = false;
    }
    // Legacy planner rejects empty snapshots. Its cache must be reset separately.
    if (cached_.width > 0) pub_->publish(cached_);
  }
  std::unique_ptr<GlobalGrid> grid_;
  std::string output_directory_, load_path_;
  bool dirty_ = true;
  sensor_msgs::msg::PointCloud2 cached_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_, load_, clear_;
};
}  // namespace rubi

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<rubi::WrapperNode>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rubi_global_heightmap_wrapper"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
