#include <gtest/gtest.h>

#include <vector>

#include "uav_nav_core/polyline.hpp"

using namespace uav_nav_core;

TEST(Polyline, LengthAndPointAlong)
{
  const std::vector<Eigen::Vector3d> pts{{0, 0, 0}, {3, 0, 0}, {3, 4, 0}};
  EXPECT_DOUBLE_EQ(polylineLength(pts), 7.0);
  EXPECT_TRUE(pointAlong(pts, -1.0).isApprox(pts.front()));
  EXPECT_TRUE(pointAlong(pts, 1.5).isApprox(Eigen::Vector3d(1.5, 0, 0)));
  EXPECT_TRUE(pointAlong(pts, 5.0).isApprox(Eigen::Vector3d(3, 2, 0)));
  EXPECT_TRUE(pointAlong(pts, 99.0).isApprox(pts.back()));
}

TEST(Polyline, Truncate)
{
  const std::vector<Eigen::Vector3d> pts{{0, 0, 0}, {3, 0, 0}, {3, 4, 0}};
  const auto a = truncate(pts, 5.0);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_TRUE(a[2].isApprox(Eigen::Vector3d(3, 2, 0)));
  const auto b = truncate(pts, 3.0);  // exactly at a vertex
  ASSERT_EQ(b.size(), 2u);
  EXPECT_TRUE(b[1].isApprox(pts[1]));
  EXPECT_EQ(truncate(pts, 99.0).size(), 3u);
  EXPECT_NEAR(polylineLength(truncate(pts, 99.0)), 7.0, 1e-12);
}

TEST(Polyline, Densify)
{
  const std::vector<Eigen::Vector3d> pts{{0, 0, 0}, {2.5, 0, 0}, {2.5, 0.4, 0}};
  const auto d = densify(pts, 1.0);
  ASSERT_EQ(d.size(), 5u);  // 3 parts of the first segment, 1 of the second, + the start
  EXPECT_TRUE(d.front().isApprox(pts.front()));
  EXPECT_TRUE(d.back().isApprox(pts.back()));
  for (size_t i = 1; i < d.size(); ++i) {EXPECT_LE((d[i] - d[i - 1]).norm(), 1.0 + 1e-12);}
  EXPECT_NEAR(polylineLength(d), polylineLength(pts), 1e-12);
  EXPECT_EQ(densify(pts, 0.0).size(), pts.size());
}
