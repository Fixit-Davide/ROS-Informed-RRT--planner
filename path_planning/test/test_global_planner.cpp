#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "path_planning/collision_world.hpp"
#include "path_planning/global_planner.hpp"
#include "path_planning/occupancy_map.hpp"

using namespace path_planning;

namespace
{

const Eigen::Vector3d kMin(-10.0, -7.0, 0.5), kMax(10.0, 7.0, 4.0);
const Eigen::Vector3d kBox(1.2, 1.2, 0.9);  // robot.size + planning_margin

// Wall x = 5.05 over y in [-7.5, 7.5], z in [0.3, 4.6], minus the gap y in [gap_lo, gap_hi].
std::shared_ptr<const OccupancySnapshot> wallMap(double gap_lo, double gap_hi)
{
  OccupancyMap map(MapOptions{});
  std::vector<Eigen::Vector3d> pts;
  for (double y = -7.5; y <= 7.5; y += 0.05) {
    if (y > gap_lo && y < gap_hi) {continue;}
    for (double z = 0.3; z <= 4.6; z += 0.05) {pts.emplace_back(5.05, y, z);}
  }
  for (int i = 0; i < 3; ++i) {map.insert(pts, {0.0, 0.0, 2.0});}
  return map.snapshot();
}

}  // namespace

TEST(Polyline, LengthAndPointAlong)
{
  const std::vector<Eigen::Vector3d> pts{{0, 0, 0}, {3, 0, 0}, {3, 4, 0}};
  EXPECT_DOUBLE_EQ(polylineLength(pts), 7.0);
  EXPECT_TRUE(pointAlong(pts, -1.0).isApprox(pts.front()));
  EXPECT_TRUE(pointAlong(pts, 1.5).isApprox(Eigen::Vector3d(1.5, 0, 0)));
  EXPECT_TRUE(pointAlong(pts, 5.0).isApprox(Eigen::Vector3d(3, 2, 0)));
  EXPECT_TRUE(pointAlong(pts, 99.0).isApprox(pts.back()));
}

TEST(GlobalPlanner, StraightLineInFreeSpace)
{
  CollisionWorld world(nullptr, kBox);
  GlobalPlanner planner(GlobalPlannerOptions{});
  const Eigen::Vector3d s(-8.0, -5.0, 1.0), g(8.3, 6.1, 3.7);
  const auto plan = planner.plan(world, kMin, kMax, s, g);
  ASSERT_TRUE(plan.success);
  ASSERT_EQ(plan.path.size(), 2u);  // shortcut to the direct segment
  EXPECT_TRUE(plan.path.front().isApprox(s));
  EXPECT_TRUE(plan.path.back().isApprox(g));
  EXPECT_NEAR(plan.length, (g - s).norm(), 1e-9);
}

TEST(GlobalPlanner, GoesThroughTheGapInAWall)
{
  CollisionWorld world(wallMap(2.0, 4.0), kBox);
  GlobalPlanner planner(GlobalPlannerOptions{});
  const Eigen::Vector3d s(0.0, -3.0, 2.0), g(8.0, -3.0, 2.0);
  const auto t0 = std::chrono::steady_clock::now();
  const auto plan = planner.plan(world, kMin, kMax, s, g);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(plan.success);
  EXPECT_TRUE(plan.path.front().isApprox(s));
  EXPECT_TRUE(plan.path.back().isApprox(g));
  EXPECT_GT(plan.length, (g - s).norm() + 4.0);  // detour of ~2 x 6 m sideways
  for (size_t i = 1; i < plan.path.size(); ++i) {
    // same sampling step as the planner's (and OMPL's) segment checks
    EXPECT_TRUE(world.isSegmentFree(plan.path[i - 1], plan.path[i], 0.1))
      << i << ": " << plan.path[i - 1].transpose() << " -> " << plan.path[i].transpose();
  }
  // It crosses x = 5 inside the gap.
  bool crossed = false;
  for (size_t i = 1; i < plan.path.size(); ++i) {
    const auto & a = plan.path[i - 1], & b = plan.path[i];
    if ((a.x() - 5.1) * (b.x() - 5.1) <= 0.0) {
      const double t = (5.1 - a.x()) / (b.x() - a.x());
      const double y = a.y() + t * (b.y() - a.y());
      EXPECT_GT(y, 2.0);
      EXPECT_LT(y, 4.0);
      crossed = true;
    }
  }
  EXPECT_TRUE(crossed);
  EXPECT_LE(plan.path.size(), 6u);
  RecordProperty("plan_ms", std::to_string(ms));
  std::printf("  gap route: %zu points, %.2f m, %zu expansions, %.1f ms\n", plan.path.size(),
    plan.length, plan.expansions, ms);
}

TEST(GlobalPlanner, FailsWhenTheWallIsClosed)
{
  CollisionWorld world(wallMap(100.0, 100.0), kBox);
  GlobalPlanner planner(GlobalPlannerOptions{});
  const auto plan = planner.plan(world, kMin, kMax, {0.0, 0.0, 2.0}, {8.0, 0.0, 2.0});
  EXPECT_FALSE(plan.success);
  EXPECT_GT(plan.expansions, 0u);
}

TEST(GlobalPlanner, StartInsideTheInflatedWallIsSnapped)
{
  CollisionWorld world(wallMap(2.0, 4.0), kBox);
  ASSERT_FALSE(world.isFree({4.6, -3.0, 2.0}));
  GlobalPlanner planner(GlobalPlannerOptions{});
  const auto plan = planner.plan(world, kMin, kMax, {4.6, -3.0, 2.0}, {-5.0, -3.0, 2.0});
  ASSERT_TRUE(plan.success);
  EXPECT_TRUE(plan.path.front().isApprox(Eigen::Vector3d(4.6, -3.0, 2.0)));
  EXPECT_TRUE(world.isFree(plan.path[1]));
}
