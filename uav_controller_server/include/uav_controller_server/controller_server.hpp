// Controller server: the only node that talks to PX4. It streams offboard setpoints at
// rate_hz (hold, takeoff, hover, path following), serves the follow_path action and the
// get_replan_start service, and checks the path ahead against the zero-copy map from the map
// server, braking before a newly blocked section and reporting it in the action feedback.
#ifndef UAV_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define UAV_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/empty.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "uav_controller_server/path_trajectory.hpp"
#include "uav_controller_server/px4_frames.hpp"
#include "uav_nav_core/collision_world.hpp"
#include "uav_nav_core/voxel_map.hpp"
#include "uav_nav_msgs/action/follow_path.hpp"
#include "uav_nav_msgs/msg/controller_state.hpp"
#include "uav_nav_msgs/srv/get_replan_start.hpp"

namespace uav_controller_server
{

class ControllerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using FollowPath = uav_nav_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;
  using State = uav_nav_msgs::msg::ControllerState;

  explicit ControllerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerServer() override = default;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // ---- callbacks (all on this node's executor thread: no locking needed) ----
  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void mapCallback(std::shared_ptr<const uav_nav_core::VoxelMap> map);
  void controlLoop();
  void blockCheck();
  void acceptPath(const std::shared_ptr<GoalHandle> & gh);
  void replanStart(
    const std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Request> req,
    std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Response> res);

  // ---- path following ----
  // Start following waypoints; while moving, the new path is spliced onto the reference.
  // Returns false if there is nothing to follow (degenerate path).
  bool startTrajectory(std::vector<Eigen::Vector3d> waypoints);
  void followTrajectory(double dt);
  void enterHover(const Eigen::Vector3d & p);
  // Ends the active goal (if any) with the given outcome.
  void finishGoal(bool success, uint16_t code, const std::string & msg, bool canceled = false);
  void publishFeedback();
  void publishState(bool force);
  void publishTrajectory();

  // ---- PX4 ----
  void publishOffboardControlMode();
  void publishSetpoint(
    const Eigen::Vector3d & pos_world, double yaw_world,
    const std::optional<Eigen::Vector3d> & vel_world = std::nullopt,
    const std::optional<Eigen::Vector3d> & acc_world = std::nullopt);
  void publishVehicleCommand(uint32_t command, float p1 = 0.f, float p2 = 0.f);
  void engageOffboardAndArm();
  bool isOffboard() const;
  bool isArmed() const;
  uint64_t px4Timestamp() const;
  void requestMapReset();

  // ---- parameters ----
  std::string map_frame_;
  WorldFrame world_frame_{WorldFrame::NWU};
  Eigen::Vector3d robot_size_;
  double min_altitude_{0.5};   // bounds.min z: above this the vehicle counts as airborne
  double control_rate_{20.0}, block_check_rate_{2.0}, check_resolution_{0.1};
  double acceptance_radius_{0.3}, takeoff_altitude_{2.0};
  TrajectoryLimits traj_limits_;
  double max_yaw_rate_{1.0}, lag_soft_{1.0}, lag_max_{2.0}, stop_margin_{0.5};
  bool auto_arm_{false}, reset_map_on_takeoff_{true};
  int target_system_{1}, target_component_{1};
  std::string t_odom_, t_status_, t_offboard_, t_setpoint_, t_command_, t_map_, reset_service_;

  // ---- ROS interfaces ----
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<uav_nav_core::VoxelMap>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp_lifecycle::LifecyclePublisher<State>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::Service<uav_nav_msgs::srv::GetReplanStart>::SharedPtr replan_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::TimerBase::SharedPtr control_timer_, block_timer_;

  // ---- map ----
  std::shared_ptr<const uav_nav_core::CollisionWorld> world_;  // robot box: path checks

  // ---- vehicle state ----
  uint8_t state_{State::WAIT_FOR_ODOM};
  bool odom_received_{false}, status_received_{false};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  double yaw_{0.0};
  uint8_t nav_state_{0}, arming_state_{0};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  double hold_yaw_{0.0};
  int offboard_counter_{0};
  bool pilot_override_{false};
  std::optional<rclcpp::Time> last_tick_, last_state_publish_;
  uint8_t last_published_state_{255};

  // ---- path following ----
  std::shared_ptr<GoalHandle> goal_;   // active follow_path goal
  bool goal_final_{true};              // its path ends at the navigation goal
  PathTrajectory traj_;
  uint64_t traj_version_{0};           // bumped whenever traj_ is rebuilt
  double s_ref_{0.0}, v_ref_{0.0};     // reference arc length [m] and speed [m/s]
  double a_ref_{0.0};                  // reference along-path acceleration [m/s^2]
  double yaw_ref_{0.0};
  std::optional<double> brake_s_;      // stop the reference before this arc length
  bool blocked_{false};                // braking before a blocked section, replan needed
  double blocked_s_{0.0};              // arc length of the blocked point
  bool stop_and_hover_{false};         // goal canceled: brake along the path, then hover
  // Where the last get_replan_start answer lies on traj_ (valid for that traj_version_ only).
  std::optional<std::pair<uint64_t, double>> replan_from_;
  int feedback_counter_{0};
};

}  // namespace uav_controller_server

#endif  // UAV_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
