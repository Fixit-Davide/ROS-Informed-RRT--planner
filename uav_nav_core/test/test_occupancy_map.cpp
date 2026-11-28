#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "uav_nav_core/collision_world.hpp"
#include "uav_nav_core/occupancy_map.hpp"

using uav_nav_core::CollisionWorld;
using uav_nav_core::MapOptions;
using uav_nav_core::OccupancyMap;

namespace
{

const Eigen::Vector3d kOrigin(0.0, 0.0, 2.0);

// Wall perpendicular to x at `x`, as a lidar at kOrigin would see it.
std::vector<Eigen::Vector3d> wall(double x)
{
  std::vector<Eigen::Vector3d> pts;
  for (double y = -3.0; y <= 3.0; y += 0.05) {
    for (double z = 0.5; z <= 4.0; z += 0.05) {pts.emplace_back(x, y, z);}
  }
  return pts;
}

Bonxai::CoordT coordOf(const Eigen::Vector3d & p, double res)
{
  return {static_cast<int32_t>(std::floor(p.x() / res)),
    static_cast<int32_t>(std::floor(p.y() / res)),
    static_cast<int32_t>(std::floor(p.z() / res))};
}

}  // namespace

TEST(OccupancyMap, WallIsOccupiedAndRaysAreFree)
{
  OccupancyMap map(MapOptions{});
  for (int i = 0; i < 3; ++i) {map.insert(wall(5.05), kOrigin);}
  const auto snap = map.snapshot();
  ASSERT_GT(snap->size(), 0u);
  EXPECT_TRUE(snap->isOccupied(coordOf({5.05, 0.0, 2.0}, 0.2)));
  EXPECT_FALSE(snap->isOccupied(coordOf({2.5, 0.0, 2.0}, 0.2)));
  for (const auto & c : snap->voxels()) {EXPECT_EQ(c.x, 25);}  // only the wall column
}

TEST(OccupancyMap, PointsOutsideZLimitsAreDropped)
{
  MapOptions opt;
  opt.min_z = 0.3;
  opt.max_z = 6.0;
  OccupancyMap map(opt);
  std::vector<Eigen::Vector3d> floor_and_sky{{3.0, 0.0, 0.1}, {3.0, 1.0, 0.29}, {3.0, 0.0, 7.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0}};
  EXPECT_EQ(map.insert(floor_and_sky, kOrigin), 0u);
  EXPECT_EQ(map.snapshot()->size(), 0u);
}

TEST(OccupancyMap, RemovedObstacleIsClearedByLaterScans)
{
  OccupancyMap map(MapOptions{});
  for (int i = 0; i < 3; ++i) {map.insert(wall(5.05), kOrigin);}
  ASSERT_TRUE(map.snapshot()->isOccupied(coordOf({5.05, 0.0, 2.0}, 0.2)));
  // The wall moved back to x = 8: rays now cross x = 5 and lower its occupancy.
  for (int i = 0; i < 10; ++i) {map.insert(wall(8.05), kOrigin);}
  const auto snap = map.snapshot();
  EXPECT_FALSE(snap->isOccupied(coordOf({5.05, 0.0, 2.0}, 0.2)));
  EXPECT_TRUE(snap->isOccupied(coordOf({8.05, 0.0, 2.0}, 0.2)));
}

TEST(OccupancyMap, ResetEmptiesTheMap)
{
  OccupancyMap map(MapOptions{});
  map.insert(wall(5.05), kOrigin);
  ASSERT_GT(map.snapshot()->size(), 0u);
  map.reset();
  EXPECT_EQ(map.snapshot()->size(), 0u);
}

TEST(CollisionWorld, EmptyMapIsFree)
{
  CollisionWorld w(nullptr, Eigen::Vector3d::Ones());
  EXPECT_FALSE(w.hasMap());
  EXPECT_TRUE(w.isFree({0, 0, 0}));
  EXPECT_EQ(w.clearance({0, 0, 0}), std::numeric_limits<double>::max());
}

TEST(CollisionWorld, BoxAgainstWall)
{
  OccupancyMap map(MapOptions{});
  for (int i = 0; i < 3; ++i) {map.insert(wall(5.05), kOrigin);}
  CollisionWorld w(map.snapshot(), Eigen::Vector3d::Ones());  // half-size 0.5
  // wall voxels span x in [5.0, 5.2)
  EXPECT_TRUE(w.isFree({2.0, 0.0, 2.0}));
  EXPECT_TRUE(w.isFree({4.5, 0.0, 2.0}));    // face touches x = 5.0
  EXPECT_FALSE(w.isFree({4.51, 0.0, 2.0}));
  EXPECT_FALSE(w.isFree({5.1, 0.0, 2.0}));
  EXPECT_TRUE(w.isFree({5.7, 0.0, 2.0}));    // behind the wall, face touches x = 5.2
  EXPECT_FALSE(w.isSegmentFree({2.0, 0.0, 2.0}, {7.0, 0.0, 2.0}, 0.1));
  EXPECT_TRUE(w.isSegmentFree({2.0, -1.0, 2.0}, {4.0, 1.0, 2.0}, 0.1));

  EXPECT_NEAR(w.clearance({3.0, 0.0, 2.0}), 1.5, 1e-9);  // box face x = 3.5
  EXPECT_EQ(w.clearance({5.0, 0.0, 2.0}), 0.0);
}

TEST(CollisionWorld, NonFiniteOrHugePositionsAreNotFree)
{
  OccupancyMap map(MapOptions{});
  map.insert(wall(5.05), kOrigin);
  CollisionWorld w(map.snapshot(), Eigen::Vector3d::Ones());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // These used to become an INT_MIN..INT_MAX voxel loop, i.e. a hang.
  EXPECT_FALSE(w.isFree({nan, 0.0, 2.0}));
  EXPECT_FALSE(w.isFree({0.0, inf, 2.0}));
  EXPECT_FALSE(w.isFree({1e12, 0.0, 2.0}));
  EXPECT_EQ(w.clearance({nan, 0.0, 2.0}), 0.0);
  EXPECT_FALSE(w.isSegmentFree({0.0, 0.0, 2.0}, {nan, 0.0, 2.0}, 0.1));
}

TEST(CollisionWorld, ClearanceMatchesBruteForce)
{
  OccupancyMap map(MapOptions{});
  for (int i = 0; i < 3; ++i) {
    map.insert(wall(5.05), kOrigin);
    map.insert(wall(-4.03), kOrigin);
  }
  const auto snap = map.snapshot();
  const Eigen::Vector3d size(1.2, 1.2, 0.9);
  CollisionWorld w(snap, size);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> ux(-8, 8), uy(-6, 6), uz(0, 5);
  const Eigen::Vector3d reach = 0.5 * size + Eigen::Vector3d::Constant(0.1);
  for (int i = 0; i < 200; ++i) {
    const Eigen::Vector3d p(ux(rng), uy(rng), uz(rng));
    double best = std::numeric_limits<double>::max();
    for (const auto & c : snap->voxels()) {
      const Eigen::Vector3d centre = (Eigen::Vector3d(c.x, c.y, c.z) + Eigen::Vector3d::Constant(0.5)) * 0.2;
      best = std::min(best, ((centre - p).cwiseAbs() - reach).cwiseMax(0.0).norm());
    }
    EXPECT_NEAR(w.clearance(p), best, 1e-9);
    EXPECT_EQ(w.isFree(p), best > 0.0) << p.transpose();
  }
}
