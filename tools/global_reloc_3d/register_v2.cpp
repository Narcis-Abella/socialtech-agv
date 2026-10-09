// Offline development caller, v2: KISS global SE(3) (or 4-DoF Quatro) then small_gicp.
// Superset of register.cpp (kept frozen for the first result). Adds explicit KISS
// configuration and dumps correspondences for evaluator-side diagnosis.
// No initial transform reaches KISS; GICP starts only from the KISS output.
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstring>
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

static void points(std::ostream& f, const std::vector<Point>& p) {
  f << '[';
  for (size_t i=0; i<p.size(); ++i) {
    if (i) f << ',';
    f << '[' << p[i].x() << ',' << p[i].y() << ',' << p[i].z() << ']';
  }
  f << ']';
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "usage: register_v2 map.xyz fit.xyz check.xyz voxel_m result.json"
                 " [--quatro] [--no-ratio] [--max-corr N] [--normal-gain g] [--fpfh-gain g]"
                 " [--linearity t] [--gicp-dist m] [--dump] [--init T.txt]\n";
    return 2;
  }
  try {
    auto target=load(argv[1]), source=load(argv[2]), check=load(argv[3]);
    const float resolution=std::stof(argv[4]);
    bool quatro=false, ratio=true, dump=false;
    int max_corr=5000;
    float normal_gain=3.0f, fpfh_gain=5.0f, linearity=1.0f;
    double gicp_dist=1.0;
    std::string init_path;
    for (int i=6; i<argc; ++i) {
      const std::string a=argv[i];
      auto next=[&]() { if (i+1>=argc) throw std::runtime_error("missing value for "+a); return std::string(argv[++i]); };
      if (a=="--quatro") quatro=true;
      else if (a=="--no-ratio") ratio=false;
      else if (a=="--dump") dump=true;
      else if (a=="--max-corr") max_corr=std::stoi(next());
      else if (a=="--normal-gain") normal_gain=std::stof(next());
      else if (a=="--fpfh-gain") fpfh_gain=std::stof(next());
      else if (a=="--linearity") linearity=std::stof(next());
      else if (a=="--gicp-dist") gicp_dist=std::stod(next());
      else if (a=="--init") init_path=next();
      else throw std::runtime_error("unknown option "+a);
    }
    // Constructor argument order: voxel, voxel sampling, quatro, linearity, max corr,
    // normal gain, fpfh gain; noise-bound gains stay at the library defaults (1.0/0.75).
    kiss_matcher::KISSMatcherConfig config(resolution, true, quatro, linearity, max_corr,
                                           normal_gain, fpfh_gain);
    config.use_ratio_test_=ratio;
    const auto begin=Clock::now();
    kiss_matcher::KISSMatcher matcher(config);
    // --init: GICP-only refinement of a candidate from the method's own global
    // search (correlative_search.py). KISS is skipped; nothing here is a truth seed.
    kiss_matcher::RegistrationSolution solution;
    if (init_path.empty()) {
      solution=matcher.estimate(source,target); // no initial transform
    }
    const auto [src_kp, tgt_kp]=matcher.getKeypointsFromFasterPFH();
    const auto [src_m, tgt_m]=matcher.getKeypointsFromInitialMatching();
    const auto cross=matcher.getInitialCorrespondences();
    const auto match_end=Clock::now();
    Eigen::Isometry3d initial=Eigen::Isometry3d::Identity();
    if (!init_path.empty()) {
      std::ifstream fi(init_path);
      for (int i=0; i<4; ++i) for (int j=0; j<4; ++j)
        if (!(fi >> initial.matrix()(i,j))) throw std::runtime_error("bad init matrix");
      solution.valid=true;
    } else if (solution.valid) {
      initial.linear()=solution.rotation;
      initial.translation()=solution.translation;
    }
    small_gicp::RegistrationResult refined;
    bool ran_gicp=false;
    if (solution.valid) {
      std::vector<Eigen::Vector3d> target_d,source_d;
      target_d.reserve(target.size()); source_d.reserve(source.size());
      for (const auto& p:target) target_d.push_back(p.cast<double>());
      for (const auto& p:source) source_d.push_back(p.cast<double>());
      small_gicp::RegistrationSetting setting;
      setting.type=small_gicp::RegistrationSetting::GICP;
      setting.downsampling_resolution=resolution;
      setting.max_correspondence_distance=gicp_dist;
      setting.num_threads=2;
      refined=small_gicp::align(target_d,source_d,initial,setting);
      ran_gicp=true;
    }
    const auto end=Clock::now();
    std::ofstream f(argv[5]);
    if (!f) throw std::runtime_error("cannot write result");
    f << std::setprecision(9) << "{\n  \"kiss_valid\": " << (solution.valid?"true":"false")
      << ",\n  \"config\": {\"voxel\": " << resolution << ", \"quatro\": " << (quatro?"true":"false")
      << ", \"ratio_test\": " << (ratio?"true":"false") << ", \"max_corr\": " << max_corr
      << ", \"normal_gain\": " << normal_gain << ", \"fpfh_gain\": " << fpfh_gain
      << ", \"linearity\": " << linearity << ", \"gicp_dist\": " << gicp_dist
      << ", \"mode\": \"" << (init_path.empty()?"kiss":"gicp_from_init") << "\"}"
      << ",\n  \"source_keypoints\": " << src_kp.size()
      << ",\n  \"target_keypoints\": " << tgt_kp.size()
      << ",\n  \"cross_checked\": " << cross.size()
      << ",\n  \"initial_matches\": " << src_m.size()
      << ",\n  \"final_correspondences\": " << matcher.getFinalCorrespondences().size()
      << ",\n  \"rotation_inliers\": " << (init_path.empty()?matcher.getNumRotationInliers():0)
      << ",\n  \"translation_inliers\": " << (init_path.empty()?matcher.getNumFinalInliers():0)
      << ",\n  \"kiss_seconds\": " << std::chrono::duration<double>(match_end-begin).count()
      << ",\n  \"total_seconds\": " << std::chrono::duration<double>(end-begin).count()
      << ",\n  \"gicp_ran\": " << (ran_gicp?"true":"false")
      << ",\n  \"gicp_converged\": " << (refined.converged?"true":"false")
      << ",\n  \"gicp_iterations\": " << refined.iterations
      << ",\n  \"gicp_inliers\": " << refined.num_inliers
      << ",\n  \"gicp_error\": " << refined.error
      << ",\n  \"source_points\": " << source.size()
      << ",\n  \"target_points\": " << target.size()
      << ",\n  \"check_points\": " << check.size()
      << ",\n  \"T_map_lidar_kiss\": ";
    matrix(f,initial);
    f << ",\n  \"T_map_lidar_refined\": ";
    matrix(f,refined.T_target_source);
    if (dump) {
      // getInitialCorrespondences() already returns (source, target) indices
      // (ROBINMatching.hpp getCrossCheckedCorrespondences undoes the size swap).
      std::vector<Point> xs, xt;
      for (const auto& [a,b]:cross) { xs.push_back(src_kp[a]); xt.push_back(tgt_kp[b]); }
      f << ",\n  \"cross_source\": "; points(f,xs);
      f << ",\n  \"cross_target\": "; points(f,xt);
      f << ",\n  \"pruned_source\": "; points(f,src_m);
      f << ",\n  \"pruned_target\": "; points(f,tgt_m);
    }
    f << "\n}\n";
    std::cout << "KISS valid=" << solution.valid << " cross=" << cross.size() << " pruned=" << src_m.size()
              << " inliers=" << matcher.getNumFinalInliers() << " GICP=" << refined.converged << "\n";
    return solution.valid ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
