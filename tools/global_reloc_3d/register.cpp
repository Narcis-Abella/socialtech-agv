// Offline development caller: KISS global SE(3) then small_gicp local refinement.
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
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

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "usage: register map.xyz fit.xyz check.xyz voxel_m result.json\n";
    return 2;
  }
  try {
    auto target=load(argv[1]), source=load(argv[2]), check=load(argv[3]);
    const float resolution=std::stof(argv[4]);
    const auto begin=Clock::now();
    kiss_matcher::KISSMatcher matcher(resolution);
    const auto solution=matcher.estimate(source,target); // no initial transform
    const auto matched=matcher.getKeypointsFromInitialMatching();
    const auto match_end=Clock::now();
    Eigen::Isometry3d initial=Eigen::Isometry3d::Identity();
    if (solution.valid) {
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
      setting.max_correspondence_distance=1.0;
      setting.num_threads=2;
      refined=small_gicp::align(target_d,source_d,initial,setting);
      ran_gicp=true;
    }
    const auto end=Clock::now();
    // Withheld points are deliberately excluded from KISS and GICP. This
    // diagnostic is not by itself an acceptance decision.
    std::ofstream f(argv[5]);
    if (!f) throw std::runtime_error("cannot write result");
    f << std::setprecision(12) << "{\n  \"kiss_valid\": " << (solution.valid?"true":"false")
      << ",\n  \"initial_matches\": " << std::get<0>(matched).size()
      << ",\n  \"final_correspondences\": " << matcher.getFinalCorrespondences().size()
      << ",\n  \"rotation_inliers\": " << matcher.getNumRotationInliers()
      << ",\n  \"translation_inliers\": " << matcher.getNumFinalInliers()
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
    f << "\n}\n";
    std::cout << "KISS valid=" << solution.valid << " matches=" << std::get<0>(matched).size()
              << " inliers=" << matcher.getNumFinalInliers() << " GICP=" << refined.converged << "\n";
    return solution.valid ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
