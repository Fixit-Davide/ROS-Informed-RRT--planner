// Both plugins loaded through pluginlib, as the planner server loads them.
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "uav_nav_core/collision_world.hpp"
#include "uav_nav_core/global_planner.hpp"
#include "uav_nav_core/local_planner.hpp"
#include "uav_nav_core/occupancy_map.hpp"

using namespace uav_nav_core;

namespace
{

// Wall x = 5.05 over y in [-7.5, 7.5], z in [0.3, 4.6], minus the gap y in (2, 4).
std::shared_ptr<const OccupancySnapshot> wallWithGap()
{
  OccupancyMap map(MapOptions{});
  std::vector<Eigen::Vector3d> pts;
  for (double y = -7.5; y <= 7.5; y += 0.05) {
    if (y > 2.0 && y < 4.0) {continue;}
    for (double z = 0.3; z <= 4.6; z += 0.05) {pts.emplace_back(5.05, y, z);}
  }
  for (int i = 0; i < 3; ++i) {map.insert(pts, {0.0, 0.0, 2.0});}
  return map.snapshot();
}

const Bounds kBounds{{-10.0, -7.0, 0.5}, {10.0, 7.0, 4.0}};

void expectFree(const CollisionWorld & w, const std::vector<Eigen::Vector3d> & path)
{
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(w.isSegmentFree(path[i - 1], path[i], 0.1)) << i;
  }
}

}  // namespace

class Plugins : public ::testing::Test
{
protected:
  void SetUp() override {node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_test");}
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

TEST_F(Plugins, AStarRouteGoesThroughTheGap)
{
  pluginlib::ClassLoader<GlobalPlanner> loader("uav_nav_core", "uav_nav_core::GlobalPlanner");
  auto astar = loader.createSharedInstance("uav_planners::AStarPlanner");
  astar->configure(node_, "global_planner");
  EXPECT_DOUBLE_EQ(node_->get_parameter("global_planner.resolution").as_double(), 0.4);

  const CollisionWorld world(wallWithGap(), {1.2, 1.2, 0.9});
  const Eigen::Vector3d s(0.0, -3.0, 2.0), g(8.0, -3.0, 2.0);
  const auto route = astar->plan(world, kBounds, s, g);
  ASSERT_GE(route.size(), 3u);
  EXPECT_TRUE(route.front().isApprox(s));
  EXPECT_TRUE(route.back().isApprox(g));
  expectFree(world, route);
}

TEST_F(Plugins, InformedRRTStarFindsTheGap)
{
  pluginlib::ClassLoader<LocalPlanner> loader("uav_nav_core", "uav_nav_core::LocalPlanner");
  auto rrt = loader.createSharedInstance("uav_planners::InformedRRTStar");
  rrt->configure(node_, "local_planner");

  auto world = std::make_shared<const CollisionWorld>(wallWithGap(), Eigen::Vector3d(1.2, 1.2, 0.9));
  const Eigen::Vector3d s(0.0, -3.0, 2.0), g(8.0, -3.0, 2.0);
  const auto path = rrt->plan(world, kBounds, s, g, 1.0, nullptr);
  ASSERT_GE(path.size(), 3u);
  EXPECT_TRUE(path.front().isApprox(s));
  EXPECT_LE((path.back() - g).norm(), 0.3 + 1e-9);
  expectFree(*world, path);
}

TEST_F(Plugins, InformedRRTStarStopsWhenCancelled)
{
  pluginlib::ClassLoader<LocalPlanner> loader("uav_nav_core", "uav_nav_core::LocalPlanner");
  auto rrt = loader.createSharedInstance("uav_planners::InformedRRTStar");
  rrt->configure(node_, "local_planner");
  // Closed wall: no solution, so only the cancel callback can end the solve early.
  OccupancyMap map(MapOptions{});
  std::vector<Eigen::Vector3d> pts;
  for (double y = -7.5; y <= 7.5; y += 0.05) {
    for (double z = 0.3; z <= 4.6; z += 0.05) {pts.emplace_back(5.05, y, z);}
  }
  for (int i = 0; i < 3; ++i) {map.insert(pts, {0.0, 0.0, 2.0});}
  auto world = std::make_shared<const CollisionWorld>(map.snapshot(), Eigen::Vector3d(1.2, 1.2, 0.9));
  const auto t0 = std::chrono::steady_clock::now();
  const auto path = rrt->plan(
    world, kBounds, {0.0, 0.0, 2.0}, {8.0, 0.0, 2.0}, 10.0,
    [t0]() {return std::chrono::steady_clock::now() - t0 > std::chrono::milliseconds(200);});
  EXPECT_TRUE(path.empty());
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(2));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
