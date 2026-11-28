// Planner server: serves the compute_path action with a global route plugin (A* by default)
// and a local path plugin (Informed-RRT* by default) along the first local.horizon metres of
// the route. The map arrives as zero-copy uav_nav_core::VoxelMap snapshots from the map server.
#ifndef UAV_PLANNER_SERVER__PLANNER_SERVER_HPP_
#define UAV_PLANNER_SERVER__PLANNER_SERVER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "uav_nav_core/collision_world.hpp"
#include "uav_nav_core/global_planner.hpp"
#include "uav_nav_core/local_planner.hpp"
#include "uav_nav_core/voxel_map.hpp"
#include "uav_nav_msgs/action/compute_path.hpp"

namespace uav_planner_server
{

class PlannerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ComputePath = uav_nav_msgs::action::ComputePath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlannerServer() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  struct Plan
  {
    uint16_t error{ComputePath::Result::NONE};
    std::string error_msg;
    std::vector<Eigen::Vector3d> path, route;
    bool final{true};
    double route_time{0.0}, local_time{0.0};
  };

  void mapCallback(std::shared_ptr<const uav_nav_core::VoxelMap> map);
  std::shared_ptr<const uav_nav_core::CollisionWorld> world() const;
  void worker();
  // Newest goal waiting, or the server is stopping: the running plan should give up.
  bool superseded() const;
  Plan plan(
    const Eigen::Vector3d & start, const Eigen::Vector3d & goal, double solve_time,
    const std::function<bool()> & cancelled);
  nav_msgs::msg::Path toPath(const std::vector<Eigen::Vector3d> & pts) const;
  void publishMarkers(const std::vector<Eigen::Vector3d> & pts);
  void stopWorker();

  // parameters
  std::string map_frame_, map_topic_;
  uav_nav_core::Bounds bounds_;
  Eigen::Vector3d plan_box_;
  bool use_global_{true}, direct_if_visible_{true}, fallback_to_route_{true};
  double horizon_{10.0}, window_margin_{2.0}, waypoint_spacing_{1.0}, solve_time_{0.5};
  double check_resolution_{0.1};

  // The loaders must outlive the plugins they created: declared first, destroyed last.
  std::unique_ptr<pluginlib::ClassLoader<uav_nav_core::GlobalPlanner>> global_loader_;
  std::unique_ptr<pluginlib::ClassLoader<uav_nav_core::LocalPlanner>> local_loader_;
  uav_nav_core::GlobalPlanner::Ptr global_;
  uav_nav_core::LocalPlanner::Ptr local_;

  rclcpp::Subscription<uav_nav_core::VoxelMap>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr route_pub_, path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Server<ComputePath>::SharedPtr action_server_;

  mutable std::mutex world_mutex_;
  std::shared_ptr<const uav_nav_core::CollisionWorld> world_;
  std::shared_ptr<const uav_nav_core::OccupancySnapshot> last_snapshot_;
  bool warned_frame_{false};

  mutable std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  std::shared_ptr<GoalHandle> pending_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

}  // namespace uav_planner_server

#endif  // UAV_PLANNER_SERVER__PLANNER_SERVER_HPP_
