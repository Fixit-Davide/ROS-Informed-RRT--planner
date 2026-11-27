// ROS 2 Informed-RRT* planner (OMPL + a Bonxai occupancy map built from the lidar) driving a
// PX4 vehicle in offboard mode over px4_msgs (PX4 Zenoh client with rmw_zenoh, or uXRCE-DDS).
#ifndef PATH_PLANNING__INFORMED_RRT_PLANNER_HPP_
#define PATH_PLANNING__INFORMED_RRT_PLANNER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "path_planning/collision_world.hpp"
#include "path_planning/occupancy_map.hpp"
#include "path_planning/path_trajectory.hpp"
#include "path_planning/px4_frames.hpp"

namespace path_planning
{

class InformedRRTPlanner : public rclcpp::Node
{
public:
  explicit InformedRRTPlanner(const rclcpp::NodeOptions & options);
  ~InformedRRTPlanner() override;

private:
  enum class FlightState { WAIT_FOR_ODOM, PRE_OFFBOARD, TAKEOFF, HOVER, PLANNING, FOLLOW_PATH };

  struct PlanRequest
  {
    Eigen::Vector3d start;
    Eigen::Vector3d goal;
    uint64_t seq;
    double solve_time;  // [s]
    int attempt;        // 1 for the first try
  };

  struct PlanResult
  {
    bool success;
    std::vector<Eigen::Vector3d> waypoints;
    uint64_t seq;
  };

  // ---- callbacks ----
  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void controlLoop();
  void replanCheck();

  // ---- planning ----
  void requestPlan(const Eigen::Vector3d & start, const Eigen::Vector3d & goal, int attempt = 1);
  // Replan while flying: keep following the current trajectory (stopping the reference
  // at brake_s if given) and plan from the point it will reach when the plan is ready.
  void replanWhileMoving(std::optional<double> brake_s, int attempt = 1);
  // Start following waypoints; while moving, the new path is spliced onto the reference.
  void startTrajectory(std::vector<Eigen::Vector3d> waypoints);
  void followTrajectory(double dt);
  void plannerWorker();
  PlanResult solve(const PlanRequest & req, std::shared_ptr<const CollisionWorld> world);
  void publishPath(const std::vector<Eigen::Vector3d> & wps);
  std::shared_ptr<const CollisionWorld> world() const;       // robot box: path checks
  std::shared_ptr<const CollisionWorld> planWorld() const;   // box + planning margin

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
  void enterHover(const Eigen::Vector3d & p);
  void resetMap();
  void updateWorlds();        // new collision snapshots from map_
  void publishOccupiedVoxels(const OccupancySnapshot & snap);

  // ---- parameters ----
  std::string map_frame_;
  WorldFrame world_frame_;
  Eigen::Vector3d bounds_min_, bounds_max_;
  double solve_time_, planner_range_, check_resolution_, waypoint_spacing_, cost_threshold_;
  double length_weight_, clearance_weight_;
  bool k_nearest_, smooth_path_;
  std::string objective_;
  double acceptance_radius_, takeoff_altitude_, default_goal_altitude_;
  TrajectoryLimits traj_limits_;
  double max_yaw_rate_, lag_soft_, lag_max_, stop_margin_;
  int max_attempts_;
  bool auto_arm_, reset_map_on_takeoff_;
  int target_system_, target_component_;
  Eigen::Vector3d robot_size_;       // collision box [m]
  Eigen::Vector3d robot_plan_size_;  // robot_size_ + planning_margin per side
  double map_publish_period_{0.5};   // [s] occupied voxel markers (0 = off)

  // ---- ROS interfaces ----
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_map_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_, replan_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;

  // ---- map (executor thread only) ----
  std::unique_ptr<OccupancyMap> map_;
  std::vector<Eigen::Vector3d> cloud_points_;
  std::optional<rclcpp::Time> last_map_publish_;
  double insert_ms_sum_{0.0}, snapshot_ms_sum_{0.0};
  size_t scans_since_log_{0}, scans_total_{0}, last_snapshot_size_{0};

  // ---- vehicle state (executor thread only) ----
  FlightState state_{FlightState::WAIT_FOR_ODOM};
  bool odom_received_{false}, status_received_{false};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  double yaw_{0.0};
  uint8_t nav_state_{0}, arming_state_{0};
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  double hold_yaw_{0.0};
  int offboard_counter_{0};
  bool pilot_override_{false};
  std::vector<Eigen::Vector3d> path_;  // = traj_.points(), checked by replanCheck()
  size_t wp_idx_{0};                   // next vertex ahead of the reference
  PathTrajectory traj_;
  double s_ref_{0.0}, v_ref_{0.0};     // reference arc length [m] and speed [m/s]
  double a_ref_{0.0};                  // reference along-path acceleration [m/s^2]
  double yaw_ref_{0.0};
  std::optional<rclcpp::Time> last_tick_;
  bool replan_pending_{false};         // a replan was requested while following the path
  std::optional<double> brake_s_;      // stop the reference before this arc length
  double replan_from_s_{0.0};          // arc length of the replan start on traj_
  bool stop_and_hover_{false};         // planning failed: brake along the path, then hover
  int current_attempt_{1};             // attempt number of the running plan request
  std::optional<Eigen::Vector3d> goal_;
  bool goal_pending_{false};

  // ---- shared with planner thread ----
  mutable std::mutex world_mutex_;
  std::shared_ptr<const CollisionWorld> world_, plan_world_;
  std::mutex plan_mutex_;
  std::condition_variable plan_cv_;
  std::optional<PlanRequest> plan_request_;
  std::optional<PlanResult> plan_result_;
  std::atomic<uint64_t> plan_seq_{0};
  std::atomic<bool> shutdown_{false};
  std::thread planner_thread_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__INFORMED_RRT_PLANNER_HPP_
