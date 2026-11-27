#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "path_planning/path_trajectory.hpp"

using path_planning::PathTrajectory;
using path_planning::TrajectoryLimits;
using Eigen::Vector3d;

namespace
{

TrajectoryLimits limits()
{
  TrajectoryLimits l;
  l.max_speed = 2.0;
  l.max_accel = 1.0;
  l.max_lateral_accel = 1.0;
  return l;
}

std::vector<Vector3d> straight(double length, double spacing)
{
  std::vector<Vector3d> pts;
  for (double x = 0.0; x <= length + 1e-9; x += spacing) {pts.emplace_back(x, 0.0, 2.0);}
  return pts;
}

}  // namespace

TEST(PathTrajectory, ArcLengthPositionTangent)
{
  const PathTrajectory t({{0, 0, 0}, {3, 0, 0}, {3, 4, 0}}, limits());
  EXPECT_DOUBLE_EQ(t.length(), 7.0);
  EXPECT_TRUE(t.position(1.5).isApprox(Vector3d(1.5, 0, 0)));
  EXPECT_TRUE(t.position(5.0).isApprox(Vector3d(3, 2, 0)));
  EXPECT_TRUE(t.position(-1.0).isApprox(Vector3d(0, 0, 0)));
  EXPECT_TRUE(t.position(99.0).isApprox(Vector3d(3, 4, 0)));
  EXPECT_TRUE(t.tangent(1.0).isApprox(Vector3d::UnitX()));
  EXPECT_TRUE(t.tangent(4.0).isApprox(Vector3d::UnitY()));
}

TEST(PathTrajectory, DropsRepeatedPointsAndDegenerates)
{
  EXPECT_TRUE(PathTrajectory({{1, 1, 1}, {1, 1, 1}}, limits()).empty());
  const PathTrajectory t({{0, 0, 0}, {0, 0, 0}, {1, 0, 0}}, limits());
  ASSERT_FALSE(t.empty());
  EXPECT_EQ(t.points().size(), 2u);
}

TEST(PathTrajectory, SpeedLimitBrakesToRestAtTheEnd)
{
  const PathTrajectory t(straight(10.0, 1.0), limits());
  EXPECT_DOUBLE_EQ(t.speedLimit(0.0), 2.0);
  EXPECT_DOUBLE_EQ(t.speedLimit(10.0), 0.0);
  // v = sqrt(2 a d) within braking distance of the end (a = 1, d = 1 m).
  EXPECT_NEAR(t.speedLimit(9.0), std::sqrt(2.0), 1e-9);
  // Monotonically non-increasing on a straight path.
  for (double s = 0.0; s < 10.0; s += 0.1) {
    EXPECT_GE(t.speedLimit(s) + 1e-12, t.speedLimit(s + 0.1));
  }
}

TEST(PathTrajectory, CornersAreSlowerThanStraights)
{
  // 90 deg corner with 1 m segments: kappa = (pi / 2) / 1 m, v = sqrt(a_lat / kappa).
  std::vector<Vector3d> pts = straight(5.0, 1.0);
  for (int i = 1; i <= 5; ++i) {pts.emplace_back(5.0, i, 2.0);}
  const PathTrajectory t(pts, limits());
  const double v_corner = std::sqrt(1.0 / (M_PI_2 / 1.0));
  EXPECT_NEAR(t.speedLimit(5.0 - 1e-9), v_corner, 1e-6);
  EXPECT_GT(t.speedLimit(2.0), v_corner);
}

TEST(PathTrajectory, ProjectAndSlice)
{
  const PathTrajectory t({{0, 0, 0}, {4, 0, 0}, {4, 4, 0}}, limits());
  EXPECT_NEAR(t.project({2.0, 1.0, 0.0}), 2.0, 1e-9);
  EXPECT_NEAR(t.project({5.0, 3.0, 0.0}), 7.0, 1e-9);
  const auto part = t.slice(2.0, 6.0);
  ASSERT_EQ(part.size(), 3u);
  EXPECT_TRUE(part[0].isApprox(Vector3d(2, 0, 0)));
  EXPECT_TRUE(part[1].isApprox(Vector3d(4, 0, 0)));
  EXPECT_TRUE(part[2].isApprox(Vector3d(4, 2, 0)));
}

// Integrating the follower law used by the planner (accelerate at a, never exceed the
// limit) reaches the end of the path, at rest, without ever exceeding max_speed.
TEST(PathTrajectory, FollowerReachesTheEndSmoothly)
{
  const auto l = limits();
  const PathTrajectory t(straight(12.0, 1.0), l);
  double s = 0.0, v = 0.0, prev_v = 0.0;
  const double dt = 0.05;
  int steps = 0;
  while (s < t.length() - 1e-6 && steps < 2000) {
    v = std::max(0.0, std::clamp(t.speedLimit(s), v - 2.0 * l.max_accel * dt, v + l.max_accel * dt));
    if (v < 0.05) {v = 0.05;}
    s = std::min(s + v * dt, t.length());
    EXPECT_LE(v, l.max_speed + 1e-9);
    EXPECT_LE(std::abs(v - prev_v), 2.0 * l.max_accel * dt + 1e-9);
    prev_v = v;
    ++steps;
  }
  EXPECT_LT(steps, 2000);
  EXPECT_DOUBLE_EQ(s, t.length());
  // Straight 12 m at 2 m/s with 1 m/s^2: about 8 s (2 s up, 4 s cruise, 2 s down).
  EXPECT_NEAR(steps * dt, 8.0, 1.0);
}
