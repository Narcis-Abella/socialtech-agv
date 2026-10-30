#pragma once
// Core of the simulated non-repetitive Mid-360: closed-form scan model on the slot clock (notes/sim_realism/livox_modelo_matematico.md), triangle scene (Embree), pose interpolation.
#include <embree4/rtcore.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace livoxsim
{
// Closed-form Mid-360 scan on the SLOT clock (notes/sim_realism/livox_modelo_matematico.md): slot k, line l = k % 4, t = k * 5 us, rotor angle a = phi0 + 2 pi f_a t,
// oscillator phase z = psi + 2 pi f_z t; azimuth = deg(a) + l + B.c_az, zenith = B.c_ze with B = [onehot(l), sin/cos(h z) h=1..3, sin/cos(h a) h=1..3, onehot(l) sin a, onehot(l) cos a].
// Text file (livox_model_to_txt.py): "f_a f_z phi0 psi" then 24 c_az then 24 c_ze.
constexpr double kSlot = 5e-6;          // s per point
constexpr int kPacket = 96;             // points per packet (480 us)

struct Model
{
  double f_a = 0, f_z = 0, phi0 = 0, psi = 0, c_az[24] = {}, c_ze[24] = {};

  void direction(uint64_t k, double & az_rad, double & ze_rad) const
  {
    const double t = double(k) * kSlot;
    const double a = phi0 + 2 * M_PI * (f_a * t - std::floor(f_a * t)), z = psi + 2 * M_PI * (f_z * t - std::floor(f_z * t));
    const int l = int(k & 3);
    double B[24] = {};
    B[l] = 1.0;
    for (int h = 1; h <= 3; ++h) {
      B[2 + 2 * h] = std::sin(h * z); B[3 + 2 * h] = std::cos(h * z);
      B[8 + 2 * h] = std::sin(h * a); B[9 + 2 * h] = std::cos(h * a);
    }
    B[16 + l] = std::sin(a); B[20 + l] = std::cos(a);
    double az = std::fmod(a * 180.0 / M_PI + l, 360.0), ze = 0;
    for (int i = 0; i < 24; ++i) {az += B[i] * c_az[i]; ze += B[i] * c_ze[i];}
    az_rad = az * M_PI / 180.0; ze_rad = ze * M_PI / 180.0;
  }
};

inline Model load_model(const std::string & path)
{
  FILE * f = std::fopen(path.c_str(), "r");
  if (!f) {throw std::runtime_error("cannot open model " + path);}
  Model m; bool ok = std::fscanf(f, "%lf %lf %lf %lf", &m.f_a, &m.f_z, &m.phi0, &m.psi) == 4;
  for (int i = 0; ok && i < 24; ++i) {ok = std::fscanf(f, "%lf", &m.c_az[i]) == 1;}
  for (int i = 0; ok && i < 24; ++i) {ok = std::fscanf(f, "%lf", &m.c_ze[i]) == 1;}
  std::fclose(f);
  if (!ok) {throw std::runtime_error("bad model file " + path);}
  return m;
}

struct Pose {double t, x, y, yaw;};

// Linear interpolation (yaw on the shortest arc); constant velocity extrapolation past the newest sample, hold before the oldest.
class PoseTrack
{
public:
  void push(const Pose & p)
  {
    if (!q_.empty() && p.t <= q_.back().t) {return;}
    q_.push_back(p);
    while (q_.size() > 2 && q_[1].t < p.t - keep_) {q_.pop_front();}
  }
  bool empty() const {return q_.empty();}
  double newest() const {return q_.back().t;}
  Pose at(double t) const
  {
    if (q_.size() == 1 || t <= q_.front().t) {Pose p = q_.front(); p.t = t; return p;}
    size_t i = q_.size() - 1;
    while (i > 1 && q_[i - 1].t > t) {--i;}
    const Pose & a = q_[i - 1], & b = q_[i];
    const double u = (t - a.t) / (b.t - a.t), dyaw = std::remainder(b.yaw - a.yaw, 2 * M_PI);
    return {t, a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.yaw + u * dyaw};
  }
private:
  std::deque<Pose> q_; double keep_ = 1.0;
};

class Scene
{
public:
  explicit Scene(const std::string & tris_path)
  {
    FILE * f = std::fopen(tris_path.c_str(), "rb");
    if (!f) {throw std::runtime_error("cannot open scene " + tris_path);}
    char magic[4]; uint32_t n = 0;
    if (std::fread(magic, 1, 4, f) != 4 || std::fread(&n, 4, 1, f) != 1 || std::string(magic, 4) != "TRIS") {std::fclose(f); throw std::runtime_error("bad scene header in " + tris_path);}
    dev_ = rtcNewDevice("threads=1");
    scene_ = rtcNewScene(dev_);
    RTCGeometry g = rtcNewGeometry(dev_, RTC_GEOMETRY_TYPE_TRIANGLE);
    auto * v = static_cast<float *>(rtcSetNewGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), 3 * size_t(n)));
    auto * ix = static_cast<unsigned *>(rtcSetNewGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), n));
    if (std::fread(v, 4, 9 * size_t(n), f) != 9 * size_t(n)) {std::fclose(f); throw std::runtime_error("short read in " + tris_path);}
    std::fclose(f);
    for (unsigned i = 0; i < 3 * n; ++i) {ix[i] = i;}
    rtcCommitGeometry(g); rtcAttachGeometry(scene_, g); rtcReleaseGeometry(g); rtcCommitScene(scene_);
    n_tri_ = n;
  }
  ~Scene() {rtcReleaseScene(scene_); rtcReleaseDevice(dev_);}
  Scene(const Scene &) = delete; Scene & operator=(const Scene &) = delete;
  size_t triangles() const {return n_tri_;}
  // distance along the unit direction in [tmin, tmax], or a negative number when nothing is hit
  float cast(const float o[3], const float d[3], float tmin, float tmax) const
  {
    RTCRayHit rh{};
    rh.ray.org_x = o[0]; rh.ray.org_y = o[1]; rh.ray.org_z = o[2]; rh.ray.dir_x = d[0]; rh.ray.dir_y = d[1]; rh.ray.dir_z = d[2];
    rh.ray.tnear = tmin; rh.ray.tfar = tmax; rh.ray.mask = ~0u; rh.ray.flags = 0; rh.hit.geomID = RTC_INVALID_GEOMETRY_ID; rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(scene_, &rh);
    return rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ? -1.0f : rh.ray.tfar;
  }
private:
  RTCDevice dev_; RTCScene scene_; size_t n_tri_ = 0;
};
}  // namespace livoxsim
