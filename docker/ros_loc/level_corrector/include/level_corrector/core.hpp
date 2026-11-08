// Pure core of level_corrector (no ROS, no registration): the roll/pitch correction C that planar_odom applies on top of the static level L (odom <- camera_init:
// L' = C * L), and the gates and filter that decide how it follows the map-registered measurements. Maths mirror SocialTech repo notes/orientacion_scripts/maptrack.py.
#pragma once
#include <Eigen/Geometry>
#include <cmath>

namespace level_corrector {
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

// Minimal rotation taking unit vector u onto unit vector v.
inline Mat3 tilt_rot(const Vec3& u, const Vec3& v) {
  const Vec3 a = u.cross(v);
  const double s = a.norm(), c = u.dot(v);
  if (s < 1e-12) return c > 0 ? Mat3::Identity() : Eigen::AngleAxisd(M_PI, Vec3::UnitX()).toRotationMatrix();   // opposite: any axis (never happens at a few degrees)
  return Eigen::AngleAxisd(std::atan2(s, c), a / s).toRotationMatrix();
}

// One step of C towards the measurement Cm: exponential with time constant tau, rotation rate limited to slew_deg_s.
inline Mat3 lowpass(const Mat3& C, const Mat3& Cm, double dt, double tau, double slew_deg_s) {
  const Eigen::AngleAxisd d(Cm * C.transpose());
  double step = d.angle() * (1.0 - std::exp(-dt / tau));
  const double lim = slew_deg_s * M_PI / 180 * dt;
  if (step > lim) step = lim;
  return Eigen::AngleAxisd(step, d.axis()).toRotationMatrix() * C;
}

// Pose of the body in map to seed the registration: gn_loc's map -> odom was fitted on the frame planar_odom publishes, which carries C, so the chain goes through C * L (without C the
// seed is off by theta * r: 0.12 m at 10 m, 0.49 m at 40 m from where FAST-LIO started, with theta = 0.7 deg).
inline Eigen::Isometry3d seed_pose(const Eigen::Isometry3d& T_map_odom, const Mat3& C, const Mat3& L, const Mat3& Rb, const Vec3& p) {
  Eigen::Isometry3d T_ob = Eigen::Isometry3d::Identity();
  T_ob.linear() = C * L * Rb;
  T_ob.translation() = C * L * p;
  return T_map_odom * T_ob;
}

// FAST-LIO restarted (its camera_init is new, so the old C belongs to nothing) from two consecutive /Odometry poses: the clock went back, or the pose moved more than a robot can in the
// gap, or it landed at the origin (where a new filter starts) from away, or there was a gap with both poses near the origin (then a reset costs centimetres and a stall cannot be told from a restart). A stall away
// from the origin (lidar covered, DDS hiccup) keeps the same camera_init and a continuous pose: not a restart, C must not drop (it would step the frame by theta * r).
inline bool restarted(double prev_t, const Vec3& prev_p, double t, const Vec3& p) {
  const double gap = t - prev_t;
  if (gap < 0) return true;
  const double jump = (p - prev_p).norm();
  if (jump > 1.0 + 1.5 * gap) return true;                                 // a pose no robot reaches in the gap
  if (p.norm() < 0.5 && jump > 0.5 + 0.5 * gap) return true;              // a new filter starts at the origin: landing there from away is a restart, driving there smoothly is not
  return gap > 1.5 && p.norm() < 2.0 && prev_p.norm() < 2.0;
}

struct Gates {
  int min_inliers = 300;        // floor: fewer points constrain neither roll nor pitch
  double min_ratio = 0.9;       // inliers / source points: density independent (good fits 0.98, bad ones 0.68-0.78 on 198 scans of 162508)
  double max_rms = 0.03;        // m
  double max_dxy = 0.3;         // m, registration vs the seed (gn_loc's planar pose)
  double max_dyaw_deg = 3.0;
};
struct Fit {
  int inliers;
  int source_points;            // after the scan voxel
  double rms;                   // m
  double dxy;                   // m
  double dyaw_deg;
};
// nullptr when the registration is trustworthy, else the name of the first gate that failed. Written as "!(x <= max)" so NaN fails.
inline const char* gate_reason(const Fit& f, const Gates& g) {
  if (f.inliers < g.min_inliers) return "inliers";
  if (!(f.inliers >= g.min_ratio * f.source_points)) return "ratio";
  if (!(f.rms <= g.max_rms)) return "rms";
  if (!(f.dxy <= g.max_dxy)) return "dxy";
  if (!(std::abs(f.dyaw_deg) <= g.max_dyaw_deg)) return "dyaw";
  return nullptr;
}

// C follows the gated measurements slowly. The first one (or the first after reset()) is taken whole; later ones that disagree
// with C by more than max_jump_deg are dropped, so a wrong registration that passed the gates cannot tilt the costmaps; C then stays where it was (= today's static level).
class Filter {
 public:
  enum Result { Primed, Accepted, Jump };
  // jump_limit: that many measurements in a row further than max_jump_deg from C (gates passed) mean C is the wrong one, e.g. a FAST-LIO restart nobody saw: the next one is taken whole.
  Filter(double tau_s, double slew_deg_s, double max_jump_deg, int jump_limit = 15) : tau_(tau_s), slew_(slew_deg_s), jump_(max_jump_deg), limit_(jump_limit) {}
  Result update(const Mat3& Cm, double t) {
    if (!primed_ || jumps_ >= limit_) { C_ = Cm; t_ = t; primed_ = true; jumps_ = 0; return Primed; }
    if (Eigen::AngleAxisd(Cm * C_.transpose()).angle() * 180 / M_PI > jump_) { jumps_++; return Jump; }
    jumps_ = 0;
    C_ = lowpass(C_, Cm, t - t_, tau_, slew_); t_ = t;
    return Accepted;
  }
  // FAST-LIO restarted: its camera_init is new, so the old C belongs to nothing. Back to the plain level until the next valid measurement, which is taken whole.
  void reset() { C_ = Mat3::Identity(); primed_ = false; jumps_ = 0; }
  const Mat3& C() const { return C_; }
  bool primed() const { return primed_; }

 private:
  double tau_, slew_, jump_, t_ = 0;
  int limit_, jumps_ = 0;
  Mat3 C_ = Mat3::Identity();
  bool primed_ = false;
};
}  // namespace level_corrector
