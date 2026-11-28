// The VoxelMap type adapter: conversions, and the intra-process path that shares the
// published object with every subscriber in the process instead of serializing it.
#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "uav_nav_core/occupancy_map.hpp"
#include "uav_nav_core/voxel_map.hpp"

using uav_nav_core::OccupancySnapshot;
using uav_nav_core::VoxelMap;
using namespace std::chrono_literals;

namespace
{

// A block of voxels, n x n x 20 cells.
std::shared_ptr<const OccupancySnapshot> block(int n)
{
  std::vector<Bonxai::CoordT> cells;
  for (int x = -n / 2; x < n / 2; ++x) {
    for (int y = 0; y < n; ++y) {
      for (int z = 0; z < 20; ++z) {cells.push_back({x, y, z});}
    }
  }
  return std::make_shared<const OccupancySnapshot>(0.2, cells);
}

template<typename Pred>
bool spinUntil(rclcpp::Executor & exec, Pred done, std::chrono::milliseconds timeout = 3s)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!done() && std::chrono::steady_clock::now() < end) {exec.spin_some(10ms);}
  return done();
}

}  // namespace

TEST(VoxelMapAdapter, RoundTrip)
{
  VoxelMap in;
  in.header.frame_id = "map";
  in.header.stamp.sec = 42;
  in.snapshot = block(10);
  uav_nav_msgs::msg::VoxelMap msg;
  uav_nav_core::toMsg(in, msg);
  EXPECT_EQ(msg.cells.size(), 3 * in.snapshot->size());
  EXPECT_DOUBLE_EQ(msg.resolution, 0.2);

  VoxelMap out;
  uav_nav_core::fromMsg(msg, out);
  ASSERT_TRUE(out.snapshot);
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_EQ(out.header.stamp.sec, 42);
  EXPECT_EQ(out.snapshot->size(), in.snapshot->size());
  for (const auto & c : in.snapshot->voxels()) {EXPECT_TRUE(out.snapshot->isOccupied(c));}
  EXPECT_FALSE(out.snapshot->isOccupied({100, 0, 0}));
  EXPECT_EQ(out.snapshot->blocks().size(), in.snapshot->blocks().size());
}

TEST(VoxelMapAdapter, MalformedMessageGivesAnEmptyMap)
{
  uav_nav_msgs::msg::VoxelMap msg;
  msg.resolution = 0.0;
  msg.cells = {1, 2, 3, 4};  // not a multiple of 3, and no resolution
  VoxelMap out;
  uav_nav_core::fromMsg(msg, out);
  ASSERT_TRUE(out.snapshot);
  EXPECT_EQ(out.snapshot->size(), 0u);
}

TEST(VoxelMapAdapter, IntraProcessSharesThePublishedObject)
{
  const auto opts = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto map_node = std::make_shared<rclcpp::Node>("map_server", opts);
  auto planner_node = std::make_shared<rclcpp::Node>("planner_server", opts);
  auto controller_node = std::make_shared<rclcpp::Node>("controller_server", opts);

  std::shared_ptr<const VoxelMap> got_planner, got_controller;
  auto sub_p = planner_node->create_subscription<VoxelMap>(
    "voxel_map", uav_nav_core::voxelMapQoS(),
    [&](std::shared_ptr<const VoxelMap> m) {got_planner = m;});
  auto sub_c = controller_node->create_subscription<VoxelMap>(
    "voxel_map", uav_nav_core::voxelMapQoS(),
    [&](std::shared_ptr<const VoxelMap> m) {got_controller = m;});
  auto pub = map_node->create_publisher<VoxelMap>("voxel_map", uav_nav_core::voxelMapQoS());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(map_node);
  exec.add_node(planner_node);
  exec.add_node(controller_node);
  ASSERT_TRUE(spinUntil(exec, [&]() {return pub->get_intra_process_subscription_count() == 2;}));

  const auto snap = block(100);  // 200 000 voxels
  auto msg = std::make_unique<VoxelMap>();
  msg->header.frame_id = "map";
  msg->snapshot = snap;
  const VoxelMap * published = msg.get();
  const auto t0 = std::chrono::steady_clock::now();
  pub->publish(std::move(msg));
  ASSERT_TRUE(spinUntil(exec, [&]() {return got_planner && got_controller;}));
  const double ipc_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

  // Both subscribers hold the very object that was published, and through it the snapshot:
  // nothing was converted, serialized or copied.
  EXPECT_EQ(got_planner.get(), published);
  EXPECT_EQ(got_controller.get(), published);
  EXPECT_EQ(got_planner->snapshot.get(), snap.get());

  // What the same map costs as a ROS message (what another process gets).
  const auto t1 = std::chrono::steady_clock::now();
  uav_nav_msgs::msg::VoxelMap ros;
  uav_nav_core::toMsg(*got_planner, ros);
  VoxelMap back;
  uav_nav_core::fromMsg(ros, back);
  const double conv_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
  std::printf(
    "  %zu voxels: intra-process publish -> 2 callbacks %.2f ms; to/from ROS message %.1f ms "
    "(%.1f MB)\n", snap->size(), ipc_ms, conv_ms, ros.cells.size() * 4.0 / 1e6);
}

TEST(VoxelMapAdapter, SubscribersWithoutIntraProcessGetTheRosMessage)
{
  auto map_node = std::make_shared<rclcpp::Node>(
    "map_server2", rclcpp::NodeOptions().use_intra_process_comms(true));
  auto echo_node = std::make_shared<rclcpp::Node>(
    "echo", rclcpp::NodeOptions().use_intra_process_comms(false));
  uav_nav_msgs::msg::VoxelMap::SharedPtr got;
  auto sub = echo_node->create_subscription<uav_nav_msgs::msg::VoxelMap>(
    "voxel_map2", uav_nav_core::voxelMapQoS(),
    [&](uav_nav_msgs::msg::VoxelMap::SharedPtr m) {got = m;});
  auto pub = map_node->create_publisher<VoxelMap>("voxel_map2", uav_nav_core::voxelMapQoS());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(map_node);
  exec.add_node(echo_node);
  ASSERT_TRUE(spinUntil(exec, [&]() {return pub->get_subscription_count() == 1;}));
  EXPECT_EQ(pub->get_intra_process_subscription_count(), 0u);

  const auto snap = block(10);
  ASSERT_TRUE(spinUntil(
      exec, [&]() {
        auto msg = std::make_unique<VoxelMap>();
        msg->snapshot = snap;
        pub->publish(std::move(msg));
        return got != nullptr;
      }));
  EXPECT_EQ(got->cells.size(), 3 * snap->size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
