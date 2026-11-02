// Offline development caller, v2: KISS global SE(3) (or 4-DoF Quatro) then small_gicp.
// Superset of register.cpp (kept frozen for the first result). Adds explicit KISS
// configuration and dumps correspondences for evaluator-side diagnosis.
// No initial transform reaches KISS; GICP starts only from the KISS output.
#include <kiss_matcher/KISSMatcher.hpp>
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

static void points(std::ostream& f, const std::vector<Point>& p) {
  f << '[';
  for (size_t i=0; i<p.size(); ++i) {
    if (i) f << ',';
    f << '[' << p[i].x() << ',' << p[i].y() << ',' << p[i].z() << ']';
  }
  f << ']';
}

// Everything one result file says. KISS-only fields stay 0 in the GICP-only modes.
struct Report {
  float resolution; bool quatro, ratio; int max_corr; float normal_gain, fpfh_gain, linearity; double gicp_dist;
  bool gicp_only, kiss_valid, ran_gicp;
  size_t src_kp=0, tgt_kp=0, cross=0, initial_matches=0, final_corr=0;
  int rotation_inliers=0, translation_inliers=0;
  double kiss_seconds=0, total_seconds=0;
  small_gicp::RegistrationResult refined;
  Eigen::Isometry3d initial;
  size_t source_points, target_points, check_points;
};

static void write_report(std::ostream& f, const Report& r) {
  f << std::setprecision(9) << "{\n  \"kiss_valid\": " << (r.kiss_valid?"true":"false")
    << ",\n  \"config\": {\"voxel\": " << r.resolution << ", \"quatro\": " << (r.quatro?"true":"false")
    << ", \"ratio_test\": " << (r.ratio?"true":"false") << ", \"max_corr\": " << r.max_corr
    << ", \"normal_gain\": " << r.normal_gain << ", \"fpfh_gain\": " << r.fpfh_gain
    << ", \"linearity\": " << r.linearity << ", \"gicp_dist\": " << r.gicp_dist
    << ", \"mode\": \"" << (r.gicp_only?"gicp_from_init":"kiss") << "\"}"
    << ",\n  \"source_keypoints\": " << r.src_kp
    << ",\n  \"target_keypoints\": " << r.tgt_kp
    << ",\n  \"cross_checked\": " << r.cross
    << ",\n  \"initial_matches\": " << r.initial_matches
    << ",\n  \"final_correspondences\": " << r.final_corr
    << ",\n  \"rotation_inliers\": " << r.rotation_inliers
    << ",\n  \"translation_inliers\": " << r.translation_inliers
    << ",\n  \"kiss_seconds\": " << r.kiss_seconds
    << ",\n  \"total_seconds\": " << r.total_seconds
    << ",\n  \"gicp_ran\": " << (r.ran_gicp?"true":"false")
    << ",\n  \"gicp_converged\": " << (r.refined.converged?"true":"false")
    << ",\n  \"gicp_iterations\": " << r.refined.iterations
    << ",\n  \"gicp_inliers\": " << r.refined.num_inliers
    << ",\n  \"gicp_error\": " << r.refined.error
    << ",\n  \"source_points\": " << r.source_points
    << ",\n  \"target_points\": " << r.target_points
    << ",\n  \"check_points\": " << r.check_points
    << ",\n  \"T_map_lidar_kiss\": ";
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
    std::cerr << "usage: register_v2 map.xyz fit.xyz check.xyz voxel_m result.json"
                 " [--quatro] [--no-ratio] [--max-corr N] [--normal-gain g] [--fpfh-gain g]"
                 " [--linearity t] [--gicp-dist m] [--dump] [--init T.txt | --init-list LIST.txt]\n"
                 "  --init-list: result.json is an existing directory; one NN.json per matrix of LIST.txt\n";
    return 2;
  }
  try {
    auto target=load(argv[1]), source=load(argv[2]), check=load(argv[3]);
    const float resolution=std::stof(argv[4]);
    bool quatro=false, ratio=true, dump=false;
    int max_corr=5000;
    float normal_gain=3.0f, fpfh_gain=5.0f, linearity=1.0f;
    double gicp_dist=1.0;
    std::string init_path, init_list_path;
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
      else if (a=="--init-list") init_list_path=next();
      else throw std::runtime_error("unknown option "+a);
    }
    if (!init_path.empty() && !init_list_path.empty()) throw std::runtime_error("--init and --init-list are exclusive");
    small_gicp::RegistrationSetting setting;
    setting.type=small_gicp::RegistrationSetting::GICP;
    setting.downsampling_resolution=resolution;
    setting.max_correspondence_distance=gicp_dist;
    setting.num_threads=2;
    Report base{resolution, quatro, ratio, max_corr, normal_gain, fpfh_gain, linearity, gicp_dist,
                true, true, true};
    base.source_points=source.size(); base.target_points=target.size(); base.check_points=check.size();
    if (!init_list_path.empty()) {
      const std::filesystem::path dir(argv[5]);
      if (!std::filesystem::is_directory(dir)) throw std::runtime_error("result must be an existing directory with --init-list");
      const int failed=refine_list(target, source, init_list_path.c_str(), dir, base, setting);
      std::cout << "GICP list failed=" << failed << "\n";
      return failed ? 1 : 0;
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
      initial=read_matrix(fi);
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
      refined=small_gicp::align(target_d,source_d,initial,setting);
      ran_gicp=true;
    }
    const auto end=Clock::now();
    std::ofstream f(argv[5]);
    if (!f) throw std::runtime_error("cannot write result");
    Report r=base;
    r.gicp_only=!init_path.empty(); r.kiss_valid=solution.valid; r.ran_gicp=ran_gicp;
    r.src_kp=src_kp.size(); r.tgt_kp=tgt_kp.size(); r.cross=cross.size(); r.initial_matches=src_m.size();
    r.final_corr=matcher.getFinalCorrespondences().size();
    r.rotation_inliers=init_path.empty()?matcher.getNumRotationInliers():0;
    r.translation_inliers=init_path.empty()?matcher.getNumFinalInliers():0;
    r.kiss_seconds=std::chrono::duration<double>(match_end-begin).count();
    r.total_seconds=std::chrono::duration<double>(end-begin).count();
    r.refined=refined; r.initial=initial;
    write_report(f,r);
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
