// GICP refinement of candidate poses, the worker of the relocalizer (search_v2.py): small_gicp from every initial transform it is given, against the map.
//   register_v2 map.xyz fit.xyz check.xyz voxel_m result [--gicp-dist m] (--init T.txt | --init-list LIST.txt)
// Until 2026-11 it was also a global registrar (KISS-Matcher without an initial transform); the correlative search replaced that, nothing in the runtime used it, and the
// dependency (not pinned, not built in the image) is gone with it (git history).
#include <small_gicp/registration/registration_helper.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Point = Eigen::Vector3f;
using Clock = std::chrono::steady_clock;

static std::vector<Point> load(const char* path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error(std::string("cannot open ") + path);
  std::vector<Point> out;
  float x, y, z;
  while (f >> x >> y >> z) {
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) out.emplace_back(x,y,z);
  }
  if (out.size() < 10) throw std::runtime_error("point cloud has fewer than 10 points");
  return out;
}

static void matrix(std::ostream& f, const Eigen::Isometry3d& T) {
  f << '[';
  for (int i=0; i<4; ++i) {
    if (i) f << ',';
    f << '[';
    for (int j=0; j<4; ++j) { if (j) f << ','; f << T.matrix()(i,j); }
    f << ']';
  }
  f << ']';
}

// Everything one result file says.
struct Report {
  float resolution; double gicp_dist;
  double total_seconds=0;
  small_gicp::RegistrationResult refined;
  Eigen::Isometry3d initial;
  size_t source_points, target_points, check_points;
};

static void write_report(std::ostream& f, const Report& r) {
  f << std::setprecision(9) << "{\n  \"config\": {\"voxel\": " << r.resolution << ", \"gicp_dist\": " << r.gicp_dist
    << ", \"mode\": \"gicp_from_init\"}"
    << ",\n  \"total_seconds\": " << r.total_seconds
    << ",\n  \"gicp_converged\": " << (r.refined.converged?"true":"false")
    << ",\n  \"gicp_iterations\": " << r.refined.iterations
    << ",\n  \"gicp_inliers\": " << r.refined.num_inliers
    << ",\n  \"gicp_error\": " << r.refined.error
    << ",\n  \"source_points\": " << r.source_points
    << ",\n  \"target_points\": " << r.target_points
    << ",\n  \"check_points\": " << r.check_points
    << ",\n  \"T_map_lidar_init\": ";
  matrix(f,r.initial);
  f << ",\n  \"T_map_lidar_refined\": ";
  matrix(f,r.refined.T_target_source);
}

static Eigen::Isometry3d read_matrix(std::istream& fi) {
  Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
  for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
    if (!(fi >> T.matrix()(i,j))) throw std::runtime_error("bad init matrix");
  return T;
}

// --init-list: GICP from every matrix of the list against one preprocessed map and source
// (the same preprocessing align() does per call, so each result equals a lone --init run).
// Result i goes to <dir>/NN.json as soon as it is done; a failed matrix leaves no file and
// does not stop the others. Returns the number of failures.
static int refine_list(const std::vector<Point>& target, const std::vector<Point>& source,
                       const char* list_path, const std::filesystem::path& dir, const Report& base,
                       const small_gicp::RegistrationSetting& setting) {
  std::ifstream fl(list_path);
  if (!fl) throw std::runtime_error(std::string("cannot open ")+list_path);
  std::vector<Eigen::Isometry3d> inits;
  while (fl >> std::ws, !fl.eof()) inits.push_back(read_matrix(fl));
  if (inits.empty()) throw std::runtime_error("empty init list");
  std::vector<Eigen::Vector3d> target_d,source_d;
  target_d.reserve(target.size()); source_d.reserve(source.size());
  for (const auto& p:target) target_d.push_back(p.cast<double>());
  for (const auto& p:source) source_d.push_back(p.cast<double>());
  const auto [target_pc, target_tree]=small_gicp::preprocess_points(target_d, setting.downsampling_resolution, 10, setting.num_threads);
  const auto [source_pc, source_tree]=small_gicp::preprocess_points(source_d, setting.downsampling_resolution, 10, setting.num_threads);
  int failed=0;
  for (size_t i=0; i<inits.size(); ++i) {
    try {
      const auto begin=Clock::now();
      Report r=base;
      r.initial=inits[i];
      r.refined=small_gicp::align(*target_pc,*source_pc,*target_tree,inits[i],setting);
      r.total_seconds=std::chrono::duration<double>(Clock::now()-begin).count();
      char name[32]; std::snprintf(name,sizeof name,"%02zu.json",i);
      const auto tmp=dir/(std::string(name)+".tmp");
      {
        std::ofstream f(tmp);
        if (!f) throw std::runtime_error("cannot write result");
        write_report(f,r);
        f << "\n}\n";
        f.flush();
        if (!f) { std::filesystem::remove(tmp); throw std::runtime_error("cannot write result"); }
      }
      std::filesystem::rename(tmp,dir/name); // a killed process never leaves half a result
    } catch (const std::exception& e) {
      std::cerr << "init " << i << ": " << e.what() << '\n';
      ++failed;
    }
  }
  return failed;
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "usage: register_v2 map.xyz fit.xyz check.xyz voxel_m result.json [--gicp-dist m] (--init T.txt | --init-list LIST.txt)\n"
                 "  --init-list: result.json is an existing directory; one NN.json per matrix of LIST.txt\n";
    return 2;
  }
  try {
    auto target=load(argv[1]), source=load(argv[2]), check=load(argv[3]);
    const float resolution=std::stof(argv[4]);
    double gicp_dist=1.0;
    std::string init_path, init_list_path;
    for (int i=6; i<argc; ++i) {
      const std::string a=argv[i];
      auto next=[&]() { if (i+1>=argc) throw std::runtime_error("missing value for "+a); return std::string(argv[++i]); };
      if (a=="--gicp-dist") gicp_dist=std::stod(next());
      else if (a=="--init") init_path=next();
      else if (a=="--init-list") init_list_path=next();
      else throw std::runtime_error("unknown option "+a);
    }
    if (init_path.empty() == init_list_path.empty()) throw std::runtime_error("exactly one of --init and --init-list is required");
    small_gicp::RegistrationSetting setting;
    setting.type=small_gicp::RegistrationSetting::GICP;
    setting.downsampling_resolution=resolution;
    setting.max_correspondence_distance=gicp_dist;
    setting.num_threads=2;
    Report base{resolution, gicp_dist};
    base.source_points=source.size(); base.target_points=target.size(); base.check_points=check.size();
    if (!init_list_path.empty()) {
      const std::filesystem::path dir(argv[5]);
      if (!std::filesystem::is_directory(dir)) throw std::runtime_error("result must be an existing directory with --init-list");
      const int failed=refine_list(target, source, init_list_path.c_str(), dir, base, setting);
      std::cout << "GICP list failed=" << failed << "\n";
      return failed ? 1 : 0;
    }
    const auto begin=Clock::now();
    std::ifstream fi(init_path);
    if (!fi) throw std::runtime_error("cannot open "+init_path);
    const Eigen::Isometry3d initial=read_matrix(fi);
    std::vector<Eigen::Vector3d> target_d,source_d;
    target_d.reserve(target.size()); source_d.reserve(source.size());
    for (const auto& p:target) target_d.push_back(p.cast<double>());
    for (const auto& p:source) source_d.push_back(p.cast<double>());
    Report r=base;
    r.initial=initial;
    r.refined=small_gicp::align(target_d,source_d,initial,setting);
    r.total_seconds=std::chrono::duration<double>(Clock::now()-begin).count();
    std::ofstream f(argv[5]);
    if (!f) throw std::runtime_error("cannot write result");
    write_report(f,r);
    f << "\n}\n";
    std::cout << "GICP=" << r.refined.converged << " inliers=" << r.refined.num_inliers << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
