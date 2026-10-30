#pragma once
// Core of the simulated non-repetitive Mid-360: pattern table (measured on a real unit, see notes/sim_realism/livox_build_pattern.py), triangle scene (Embree), pose interpolation.
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
struct Pattern
{
  double f_z = 0, f_a = 0, period = 0.1;
  std::vector<uint32_t> start;          // frame k owns points [start[k], start[k+1])
  std::vector<float> toff;              // seconds from the first point of the frame
  std::vector<uint16_t> az, ze;         // deg * 65536/360, deg * 65536/180 (zenith from +z)
  std::vector<uint8_t> line;            // laser 0..3
  size_t frames() const {return start.size() - 1;}
};

inline Pattern load_pattern(const std::string & path)
{
  FILE * f = std::fopen(path.c_str(), "rb");
  if (!f) {throw std::runtime_error("cannot open pattern " + path);}
  auto rd = [&](void * p, size_t sz, size_t n) {if (std::fread(p, sz, n, f) != n) {std::fclose(f); throw std::runtime_error("short read in " + path);}};
  char magic[4]; uint32_t hdr[3]; double d[3]; Pattern p;
  rd(magic, 1, 4); rd(hdr, 4, 3); rd(d, 8, 3);
  if (std::string(magic, 4) != "MIDP" || hdr[0] != 1) {std::fclose(f); throw std::runtime_error("bad pattern header in " + path);}
  p.f_z = d[0]; p.f_a = d[1]; p.period = d[2];
  p.start.resize(hdr[1] + 1); rd(p.start.data(), 4, hdr[1] + 1);
  const size_t n = hdr[2];
  if (p.start.back() != n) {std::fclose(f); throw std::runtime_error("inconsistent pattern " + path);}
  p.toff.resize(n); p.az.resize(n); p.ze.resize(n); p.line.resize(n);
  rd(p.toff.data(), 4, n); rd(p.az.data(), 2, n); rd(p.ze.data(), 2, n); rd(p.line.data(), 1, n);
  std::fclose(f);
  return p;
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
