// gn_offline: the gn_loc core (gn_core.hpp, the same code as the node) run straight over a replay bag (make_replay_bag.py output: /scan + odom->base_footprint in /tf), no ROS
// graph, no clock: every scan is processed in order with the odometry interpolated at its stamp (what the node's tf2 lookup does). Deterministic and faster than 1x, for studying
// the LOST / false-lock logic over many bags; the node itself is still what the 1x replays measure.
// usage: gn_offline BAG_DIR MAP_YAML "x y yaw_rad sxy syaw_rad" OUT_TF_CSV OUT_DIAG_CSV [key=value ...]   (keys: the Params fields, angles in rad)
// OUT_TF_CSV: rows as tf_rec.py writes them (stamp, mo|ob, x, y, yaw): map->odom per scan at scan stamp + 0.2 s (the node's tf_tolerance), every odom->base_footprint of the bag.
// OUT_DIAG_CSV: per scan t, state (0 WAITING_POSE, 1 TRACKING, 2 LOST), inlier ratio, consistency, need_reloc, update wall time (ms).
#include <chrono>
#include <cstdio>
#include <map>
#include <sstream>
#include <string>

#include "gn_loc/gn_core.hpp"
#include "nav2_map_server/map_io.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{
double stampOf(const builtin_interfaces::msg::Time & s) {return s.sec + s.nanosec * 1e-9;}
double yawOf(const geometry_msgs::msg::Quaternion & q) {return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));}

void setParam(gn_loc::Params & p, const std::string & k, double v)
{
  std::map<std::string, double *> d{{"sigma", &p.sigma}, {"dmax", &p.dmax}, {"robust_c", &p.robust_c}, {"q_pos", &p.q_pos}, {"q_yaw", &p.q_yaw}, {"min_inliers", &p.min_inliers},
    {"tight", &p.tight}, {"min_tight", &p.min_tight}, {"accept_tight", &p.accept_tight}, {"min_range_cv", &p.min_range_cv}, {"lost_s", &p.lost_s}, {"recover_period", &p.recover_period}, {"recover_sxy0", &p.recover_sxy0},
    {"recover_syaw0", &p.recover_syaw0}, {"recover_grow_xy", &p.recover_grow_xy}, {"recover_grow_yaw", &p.recover_grow_yaw}, {"recover_sxy_max", &p.recover_sxy_max},
    {"recover_syaw_max", &p.recover_syaw_max}, {"reloc_after_s", &p.reloc_after_s},
    {"still_window_s", &p.still_window_s}, {"still_xy", &p.still_xy}, {"still_yaw", &p.still_yaw}};
  if (k == "n_beams") {p.n_beams = int(v); return;}
  if (k == "recover_k") {p.recover_k = int(v); return;}
  if (k == "init_search") {p.init_search = v != 0; return;}
  const auto it = d.find(k);
  if (it == d.end()) {throw std::runtime_error("unknown parameter " + k);}
  *it->second = v;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 6) {std::fprintf(stderr, "usage: gn_offline BAG_DIR MAP_YAML \"x y yaw sxy syaw\" OUT_TF_CSV OUT_DIAG_CSV [key=value ...]\n"); return 2;}
  gn_loc::Params prm;
  for (int i = 6; i < argc; ++i) {
    const std::string a = argv[i]; const auto e = a.find('=');
    if (e == std::string::npos) {std::fprintf(stderr, "bad argument %s\n", argv[i]); return 2;}
    setParam(prm, a.substr(0, e), std::stod(a.substr(e + 1)));
  }
  double ix, iy, iyaw, sxy, syaw;
  std::istringstream(argv[3]) >> ix >> iy >> iyaw >> sxy >> syaw;

  nav_msgs::msg::OccupancyGrid map;
  if (nav2_map_server::loadMapFromYaml(argv[2], map) != nav2_map_server::LOAD_MAP_SUCCESS) {std::fprintf(stderr, "cannot load %s\n", argv[2]); return 1;}
  const auto & mi = map.info;
  gn_loc::Core core(std::make_shared<const gn_loc::Field>(map.data, int(mi.width), int(mi.height), mi.resolution, mi.origin.position.x, mi.origin.position.y), prm);

  rosbag2_cpp::Reader rd;
  rd.open(argv[1]);
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> sscan;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> stf;
  std::vector<std::pair<double, gn_loc::Pose>> odom;
  std::vector<sensor_msgs::msg::LaserScan> scans;
  while (rd.has_next()) {
    const auto m = rd.read_next();
    rclcpp::SerializedMessage raw(*m->serialized_data);
    if (m->topic_name == "/scan") {
      sensor_msgs::msg::LaserScan s; sscan.deserialize_message(&raw, &s); scans.push_back(std::move(s));
    } else if (m->topic_name == "/tf") {
      tf2_msgs::msg::TFMessage t; stf.deserialize_message(&raw, &t);
      for (const auto & x : t.transforms) {
        if (x.header.frame_id == "odom" && x.child_frame_id == "base_footprint") {
          odom.push_back({stampOf(x.header.stamp), {x.transform.translation.x, x.transform.translation.y, yawOf(x.transform.rotation)}});
        }
      }
    }
  }
  std::sort(odom.begin(), odom.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
  if (odom.size() < 2 || scans.empty()) {std::fprintf(stderr, "no odometry or no scans in %s\n", argv[1]); return 1;}

  FILE * ftf = std::fopen(argv[4], "w"), * fd = std::fopen(argv[5], "w");
  if (!ftf || !fd) {std::fprintf(stderr, "cannot write the outputs\n"); return 1;}
  for (const auto & [t, o] : odom) {std::fprintf(ftf, "%.4f,ob,%.5f,%.5f,%.6f\n", t, o.x, o.y, o.yaw);}
  core.setInitial({ix, iy, iyaw}, sxy, syaw);
  size_t used = 0;
  for (const auto & s : scans) {
    const double t = stampOf(s.header.stamp);
    if (t < odom.front().first || t > odom.back().first) {continue;}   // the node's MessageFilter would wait for (or drop) these: no odometry at the stamp
    const auto it = std::lower_bound(odom.begin(), odom.end(), t, [](const auto & a, double v) {return a.first < v;});
    gn_loc::Pose o = it->second;
    if (it != odom.begin() && it->first > t) {   // linear between the two odometry samples around t (yaw on the shortest arc)
      const auto & [ta, a] = *(it - 1); const auto & [tb, b] = *it; const double w = (t - ta) / (tb - ta);
      o = {a.x + w * (b.x - a.x), a.y + w * (b.y - a.y), a.yaw + w * gn_loc::wrap(b.yaw - a.yaw)};
    }
    const auto c0 = std::chrono::steady_clock::now();
    const auto r = core.update(t, o, s.ranges, s.angle_min, s.angle_increment);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count();
    if (r.Tmo) {std::fprintf(ftf, "%.4f,mo,%.5f,%.5f,%.6f\n", t + 0.2, r.Tmo->x, r.Tmo->y, r.Tmo->yaw);}
    std::fprintf(fd, "%.4f,%d,%.4f,%.4f,%d,%.3f\n", t, int(r.state), r.ratio, r.tight, int(r.need_reloc), ms);
    ++used;
  }
  std::fclose(ftf); std::fclose(fd);
  std::printf("%zu scans of %zu processed, %zu odometry samples\n", used, scans.size(), odom.size());
  return 0;
}
