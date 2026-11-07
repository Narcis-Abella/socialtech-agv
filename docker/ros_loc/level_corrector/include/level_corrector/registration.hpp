// Registration of one body-frame scan against the levelled 3D map with small_gicp point-to-plane ICP (map: voxels + normals built once). Two passes as the offline reference
// (notes/orientacion_scripts/mapreg.py, bench_track.cpp): correspondences < 0.5 m, then < 0.15 m. Single thread, no OpenMP/TBB: it must cost a small share of one core.
#pragma once
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/factors/plane_icp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/normal_estimation.hpp>

#include <limits>
#include <memory>
#include <vector>

#include "level_corrector/core.hpp"

namespace level_corrector {

struct Aligned {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();   // map <- body
  Fit fit{0, 0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0};
  bool converged = false;
};

class MapRegistrar {
 public:
  // map_points: the levelled map (floor on z = 0), already one point per voxel of `voxel` m or not: it is voxelized here. ds: voxel of the scan.
  explicit MapRegistrar(const std::vector<Eigen::Vector3f>& map_points, double voxel = 0.1, double ds = 0.25) : ds_(ds) {
    target_ = small_gicp::voxelgrid_sampling(small_gicp::PointCloud(map_points), voxel);
    tree_ = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target_);
    small_gicp::estimate_normals(*target_, *tree_, 20);
  }
  size_t map_size() const { return target_->size(); }

  Aligned align(const std::vector<Eigen::Vector3f>& body_points, const Eigen::Isometry3d& seed) const {
    Aligned out;
    if (body_points.size() < 10) return out;   // fit stays at 0 inliers: the gates reject it
    const auto src = small_gicp::voxelgrid_sampling(small_gicp::PointCloud(body_points), ds_);
    small_gicp::Registration<small_gicp::PointToPlaneICPFactor, small_gicp::SerialReduction> reg;
    reg.optimizer.max_iterations = 20;
    reg.rejector.max_dist_sq = 0.5 * 0.5;
    auto r = reg.align(*target_, *src, *tree_, seed);
    reg.rejector.max_dist_sq = 0.15 * 0.15;
    r = reg.align(*target_, *src, *tree_, r.T_target_source);
    out.fit.source_points = static_cast<int>(src->size());
    out.T = r.T_target_source;
    out.converged = r.converged;
    if (r.num_inliers > 0) {
      out.fit.inliers = static_cast<int>(r.num_inliers);
      out.fit.rms = std::sqrt(2.0 * r.error / r.num_inliers);   // PointToPlaneICPFactor::error = 0.5 * (n . d)^2
    }
    const Vec3 dp = out.T.translation() - seed.translation();
    out.fit.dxy = std::hypot(dp.x(), dp.y());
    const double dyaw = std::atan2(out.T.linear()(1, 0), out.T.linear()(0, 0)) - std::atan2(seed.linear()(1, 0), seed.linear()(0, 0));
    out.fit.dyaw_deg = std::remainder(dyaw, 2 * M_PI) * 180 / M_PI;
    return out;
  }

 private:
  double ds_;
  std::shared_ptr<small_gicp::PointCloud> target_;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree_;
};
}  // namespace level_corrector
