// Navigator: turns a goal (nav2_msgs/NavigateToPose action, or a PoseStamped on /goal_pose
// from RViz) into compute_path requests to the planner server and follow_path goals to the
// controller server, the way Nav2's bt_navigator drives its servers:
//
//  - waits for the controller to hover (takeoff) before the first plan;
//  - plans the next local segment while the current one still has local.replan_distance
//    left, from its end, so the vehicle never stops between segments;
//  - replans from just ahead of the reference when the controller reports the path blocked,
//    or when a new goal comes in while flying;
//  - retries failed plans with twice the solve time, up to max_attempts, then has the
//    controller stop along the path (cancel) and aborts the navigation.
#ifndef UAV_NAVIGATOR__NAVIGATOR_HPP_
#define UAV_NAVIGATOR__NAVIGATOR_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "uav_nav_msgs/action/compute_path.hpp"
#include "uav_nav_msgs/action/follow_path.hpp"
#include "uav_nav_msgs/msg/controller_state.hpp"
#include "uav_nav_msgs/srv/get_replan_start.hpp"

namespace uav_navigator
{

class Navigator : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using ComputePath = uav_nav_msgs::action::ComputePath;
  using FollowPath = uav_nav_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using GetReplanStart = uav_nav_msgs::srv::GetReplanStart;
  using ControllerState = uav_nav_msgs::msg::ControllerState;

  explicit Navigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Navigator() override = default;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Why a plan is requested: decides where it starts and which requests may replace it.
  enum class PlanKind { FROM_HOVER, NEW_GOAL, NEXT_SEGMENT, BLOCKED };

  // ---- goals ----
  void goalTopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void acceptNavigation(const std::shared_ptr<NavGoalHandle> & gh);
  // Goal in map_frame, z defaulted and clamped to the bounds; nullopt if it cannot be used.
  std::optional<Eigen::Vector3d> toMapGoal(const geometry_msgs::msg::PoseStamped & pose);
  void startNavigation(const Eigen::Vector3d & goal);
  void finishNavigation(bool success, uint16_t code, const std::string & msg);

  // ---- planning / following ----
  void stateCallback(const ControllerState::SharedPtr msg);
  void requestPlan(PlanKind kind, int attempt = 1);
  void onPlanResult(uint64_t seq, const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r);
  void onPlanFailed(const std::string & why);
  void sendFollow(const nav_msgs::msg::Path & path, bool final);
  void onFollowFeedback(
    FollowGoalHandle::SharedPtr gh, const std::shared_ptr<const FollowPath::Feedback> fb);
  void onFollowResult(const FollowGoalHandle::WrappedResult & r);
  void stopVehicle();  // cancel the follow_path goal: brake along the path, hover
  void tick();         // navigation cancel, NavigateToPose feedback
  double solveTime(int attempt) const;

  // ---- parameters ----
  std::string map_frame_, goal_topic_, compute_path_action_, follow_path_action_;
  std::string replan_service_, controller_state_topic_;
  Eigen::Vector3d bounds_min_, bounds_max_;
  double default_goal_altitude_{2.0}, solve_time_{0.5}, replan_distance_{5.0};
  int max_attempts_{3};

  // ---- ROS interfaces ----
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ControllerState>::SharedPtr state_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp_action::Client<ComputePath>::SharedPtr plan_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp::Client<GetReplanStart>::SharedPtr replan_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- navigation state (all on this node's executor thread) ----
  std::optional<Eigen::Vector3d> goal_;
  uint64_t nav_id_{0};            // bumped for every new goal
  uint64_t follow_nav_id_{0};     // goal the path being flown belongs to
  std::shared_ptr<NavGoalHandle> nav_goal_;  // null for goals from the topic
  rclcpp::Time nav_start_{0, 0, RCL_ROS_TIME};
  bool waiting_for_hover_{false};
  std::optional<ControllerState> controller_state_;
  uint64_t plan_seq_{0};          // id of the newest plan request; older answers are ignored
  bool plan_in_flight_{false};
  PlanKind plan_kind_{PlanKind::FROM_HOVER};
  int attempt_{1};
  bool segment_final_{true};      // the path being flown ends at the goal
  bool follow_pending_{false};    // follow_path goal sent, not accepted yet
  FollowGoalHandle::SharedPtr follow_goal_;
  double distance_remaining_{0.0};
  int16_t replans_{0};
};

}  // namespace uav_navigator

#endif  // UAV_NAVIGATOR__NAVIGATOR_HPP_
