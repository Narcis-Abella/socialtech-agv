#include <gtest/gtest.h>

#include <cstdlib>

#include "level_corrector/map_io.hpp"

using namespace level_corrector;

static std::string tmpdir() { char t[] = "/tmp/lc_mapio_XXXXXX"; return mkdtemp(t); }
static void write(const std::string& p, const std::string& s) { std::ofstream(p, std::ios::binary) << s; }
static std::string ply(const std::vector<float>& xyz, const std::string& props = "property float x\nproperty float y\nproperty float z\n") {
  std::string h = "ply\nformat binary_little_endian 1.0\nelement vertex " + std::to_string(xyz.size() / 3) + "\n" + props + "end_header\n";
  return h + std::string(reinterpret_cast<const char*>(xyz.data()), xyz.size() * sizeof(float));
}
static const char* kYaml = "image: map.pgm\nresolution: 0.025\n# T_map_world (row-major; p_map = T p_ply):\n# [0.0, -1.0, 0.0, 1.0]\n# [1.0, 0.0, 0.0, 2.0]\n# [0.0, 0.0, 1.0, 0.5]\n# [0.0, 0.0, 0.0, 1.0]\n";

TEST(MapIo, PointsAreMovedByTMapWorld) {
  const auto d = tmpdir(); write(d + "/map.yaml", kYaml); write(d + "/map.ply", ply({1, 0, 0, 0, 2, 3}));
  const auto p = read_ply_xyz(d + "/map.ply", read_t_map_world(d + "/map.yaml"));
  ASSERT_EQ(p.size(), 2u);
  EXPECT_NEAR((p[0] - Eigen::Vector3f(1, 3, 0.5)).norm(), 0, 1e-6);    // (1,0,0) -> (0*1-0+1, 1*1+2, .5)
  EXPECT_NEAR((p[1] - Eigen::Vector3f(-1, 2, 3.5)).norm(), 0, 1e-6);
}
TEST(MapIo, AYamlWithoutTheTransformIsRefused) {
  const auto d = tmpdir(); write(d + "/map.yaml", "image: map.pgm\n");
  EXPECT_THROW(read_t_map_world(d + "/map.yaml"), std::runtime_error);
}
TEST(MapIo, APlyWithOtherPropertiesOrTruncatedIsRefused) {
  const auto d = tmpdir();
  write(d + "/a.ply", ply({0, 0, 0}, "property float x\nproperty float y\nproperty float z\nproperty uchar r\n"));
  EXPECT_THROW(read_ply_xyz(d + "/a.ply", Eigen::Matrix4d::Identity()), std::runtime_error);
  std::string t = ply({1, 2, 3, 4, 5, 6}); t.resize(t.size() - 4); write(d + "/b.ply", t);
  EXPECT_THROW(read_ply_xyz(d + "/b.ply", Eigen::Matrix4d::Identity()), std::runtime_error);
  EXPECT_THROW(read_ply_xyz(d + "/none.ply", Eigen::Matrix4d::Identity()), std::runtime_error);
}
