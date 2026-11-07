// Reads the levelled 3D map the live stack already has: <dir>/map.ply (binary little-endian, float x y z) moved into the map frame by the T_map_world rows that
// make_map.sh writes as "# [a, b, c, d]" comments in <dir>/map.yaml (same as notes/orientacion_scripts/mapreg.py read_map).
#pragma once
#include <Eigen/Dense>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace level_corrector {

inline Eigen::Matrix4d read_t_map_world(const std::string& yaml_path) {
  std::ifstream f(yaml_path);
  if (!f) throw std::runtime_error("cannot open " + yaml_path);
  Eigen::Matrix4d T;
  int rows = 0;
  for (std::string l; std::getline(f, l) && rows < 4;) {
    if (l.rfind("# [", 0) != 0) continue;
    double a, b, c, d;
    if (std::sscanf(l.c_str(), "# [%lf, %lf, %lf, %lf]", &a, &b, &c, &d) != 4) throw std::runtime_error("bad T_map_world row in " + yaml_path + ": " + l);
    T.row(rows++) << a, b, c, d;
  }
  if (rows != 4) throw std::runtime_error(yaml_path + " has no T_map_world (a map from ops/make_map.sh writes it)");
  return T;
}

inline std::vector<Eigen::Vector3f> read_ply_xyz(const std::string& path, const Eigen::Matrix4d& T) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::string l; size_t n = 0; int props = 0; bool bin = false;
  while (std::getline(f, l)) {
    if (l == "format binary_little_endian 1.0") bin = true;
    else if (l.rfind("element vertex ", 0) == 0) n = std::stoul(l.substr(15));
    else if (l.rfind("property ", 0) == 0) { if (l != "property float x" && l != "property float y" && l != "property float z") throw std::runtime_error(path + ": only float x y z properties are supported, got '" + l + "'"); props++; }
    else if (l == "end_header") break;
  }
  if (!bin || props != 3 || n == 0) throw std::runtime_error(path + ": expected a binary little-endian PLY with exactly float x, y, z");
  std::vector<float> raw(3 * n);
  f.read(reinterpret_cast<char*>(raw.data()), raw.size() * sizeof(float));
  if (static_cast<size_t>(f.gcount()) != raw.size() * sizeof(float)) throw std::runtime_error(path + ": truncated");
  std::vector<Eigen::Vector3f> out(n);
  const Eigen::Matrix3f R = T.topLeftCorner<3, 3>().cast<float>(); const Eigen::Vector3f t = T.topRightCorner<3, 1>().cast<float>();
  for (size_t i = 0; i < n; i++) out[i] = R * Eigen::Vector3f(raw[3 * i], raw[3 * i + 1], raw[3 * i + 2]) + t;
  return out;
}
}  // namespace level_corrector
