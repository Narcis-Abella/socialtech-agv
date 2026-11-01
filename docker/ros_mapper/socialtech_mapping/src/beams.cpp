// C entry point of the beam counting, loaded by tools/slam_bench/ghosts.py with ctypes.
// Reads the bag once per call (rosbag2_cpp: mcap or sqlite3), decodes the scans in batches and counts them on all cores.
#include <omp.h>

#include <memory>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "socialtech_mapping/beams.hpp"

using namespace socialtech_mapping;

namespace {

// xyz of a sensor_msgs/PointCloud2, read as float32 at the x, y, z field offsets (as tools/slam_bench/beams_oracle.py does); non-finite points dropped.
double decode_cloud(const rclcpp::SerializedMessage& raw, std::vector<float>& xyz) {
  static const rclcpp::Serialization<sensor_msgs::msg::PointCloud2> ser;
  sensor_msgs::msg::PointCloud2 msg;
  ser.deserialize_message(&raw, &msg);
  long off[3] = {-1, -1, -1};
  for (const auto& f : msg.fields)
    for (int k = 0; k < 3; ++k)
      if (f.name == std::string(1, "xyz"[k])) off[k] = f.offset;
  if (off[0] < 0 || off[1] < 0 || off[2] < 0) throw std::runtime_error("PointCloud2 without x, y, z fields");
  xyz.clear();
  for (size_t o = 0; o + msg.point_step <= msg.data.size(); o += msg.point_step) {
    float p[3];
    for (int k = 0; k < 3; ++k) std::memcpy(&p[k], msg.data.data() + o + off[k], 4);
    if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) xyz.insert(xyz.end(), p, p + 3);
  }
  return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
}

}  // namespace

extern "C" {

// hit / passed (n_c each, zeroed here) per candidate C (n_c x 3, PLY frame) over every scan of `topic` that has a pose.
// poses: n_poses x 8 (t x y z qx qy qz qw). params: extr x y z, margin, rmax, min_range, el_lo, el_hi, scan_mid, max_gap.
// Returns the number of scans counted, or -1 with the reason in err.
long smap_count_beams(const char* bag, const char* topic, const double* poses, long n_poses, const double* C, long n_c,
                      const double* params, int32_t* hit, int32_t* passed, char* err, long err_len) {
  try {
    const Params P{{params[0], params[1], params[2]}, params[3], params[4], params[5], params[6], params[7], params[8], params[9]};
    const CandidateGrid grid(C, n_c, 1.0);
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    std::string type;
    for (const auto& t : reader.get_all_topics_and_types())
      if (t.name == topic) type = t.type;
    if (type.empty()) throw std::runtime_error(std::string("topic ") + topic + " is not in the bag");
    const bool custom = type == "livox_ros_driver2/msg/CustomMsg";
    if (!custom && type != "sensor_msgs/msg/PointCloud2") throw std::runtime_error("unsupported message type " + type);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic};
    reader.set_filter(filter);

    const int nt = omp_get_max_threads();
    std::vector<std::vector<int32_t>> h(nt, std::vector<int32_t>(n_c, 0)), p(nt, std::vector<int32_t>(n_c, 0));
    constexpr size_t kBatch = 64;   // scans decoded (one thread) before they are counted (all threads)
    std::vector<std::vector<float>> xyz(kBatch);
    std::vector<Pose> pose(kBatch);
    long used = 0;
    while (reader.has_next()) {
      size_t nb = 0;
      while (nb < kBatch && reader.has_next()) {
        auto m = reader.read_next();
        double stamp;
        if (custom) {
          stamp = decode_custom(m->serialized_data->buffer, m->serialized_data->buffer_length, xyz[nb]);
        } else {
          const rclcpp::SerializedMessage raw(*m->serialized_data);
          stamp = decode_cloud(raw, xyz[nb]);
        }
        if (auto q = pose_at(poses, n_poses, stamp + P.scan_mid, P.max_gap)) pose[nb++] = *q;
      }
      used += static_cast<long>(nb);
#pragma omp parallel
      {
        std::vector<double> img;
        const int t = omp_get_thread_num();
#pragma omp for schedule(dynamic, 1)
        for (long i = 0; i < static_cast<long>(nb); ++i) count_scan(grid, xyz[i], pose[i], P, img, h[t].data(), p[t].data());
      }
    }
    for (long k = 0; k < n_c; ++k) {
      const long i = grid.index(k);
      hit[i] = passed[i] = 0;
      for (int t = 0; t < nt; ++t) { hit[i] += h[t][k]; passed[i] += p[t][k]; }
    }
    return used;
  } catch (const std::exception& e) {
    std::snprintf(err, static_cast<size_t>(err_len), "%s", e.what());
    return -1;
  }
}

}  // extern "C"
