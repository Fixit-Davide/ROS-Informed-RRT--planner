#include "path_planning/informed_rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <Eigen/Geometry>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.hpp>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

namespace path_planning
{

namespace
{

Eigen::Vector3d toVec(const ob::State * s)
{
  const auto * rv = s->as<ob::RealVectorStateSpace::StateType>();
  return {rv->values[0], rv->values[1], rv->values[2]};
}

Eigen::Vector3d toVec(const std::vector<double> & v)
{
  if (v.size() != 3) {throw std::invalid_argument("expected a 3-element vector parameter");}
  return {v[0], v[1], v[2]};
}

// Penalises states close to obstacles: cost = 1 / (clearance + eps), integrated along the path.
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
  ClearanceObjective(const ob::SpaceInformationPtr & si, std::shared_ptr<const CollisionWorld> w)
  : ob::StateCostIntegralObjective(si, true), world_(std::move(w)) {}

  ob::Cost stateCost(const ob::State * s) const override
  {
    return ob::Cost(1.0 / (std::min(world_->clearance(toVec(s)), 10.0) + 0.1));
  }

private:
  std::shared_ptr<const CollisionWorld> world_;
};

}  // namespace

InformedRRTPlanner::InformedRRTPlanner(const rclcpp::NodeOptions & options)
: Node("path_planning", options)
{
  // ---------------- parameters ----------------
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  world_frame_ = worldFrameFromString(declare_parameter<std::string>("frame_convention", "nwu"));

  const auto robot_size = toVec(declare_parameter<std::vector<double>>(
      "robot.size", {0.55, 0.55, 0.50}));
  robot_size_ = robot_size;
  // Paths are planned with a slightly larger box than the one used to check them, so a
  // few new voxels next to a freshly planned path do not immediately trigger a replan.
  const double margin = declare_parameter<double>("robot.planning_margin", 0.15);
  robot_plan_size_ = robot_size + Eigen::Vector3d::Constant(2.0 * margin);

  MapOptions map_opt;
  map_opt.resolution = declare_parameter<double>("mapping.resolution", 0.2);
  map_opt.max_range = declare_parameter<double>("mapping.max_range", 15.0);
  map_opt.min_z = declare_parameter<double>("mapping.min_z", 0.3);
  map_opt.max_z = declare_parameter<double>("mapping.max_z", 6.0);
  map_opt.prob_hit = declare_parameter<double>("mapping.sensor_model.hit", 0.7);
  map_opt.prob_miss = declare_parameter<double>("mapping.sensor_model.miss", 0.4);
  map_opt.clamp_min = declare_parameter<double>("mapping.sensor_model.min", 0.12);
  map_opt.clamp_max = declare_parameter<double>("mapping.sensor_model.max", 0.97);
  map_opt.exact_rays = declare_parameter<bool>("mapping.exact_rays", true);
  const double map_publish_rate = declare_parameter<double>("mapping.publish_rate_hz", 2.0);
  map_publish_period_ = map_publish_rate > 0.0 ? 1.0 / map_publish_rate : 0.0;
  map_ = std::make_unique<OccupancyMap>(map_opt);

  bounds_min_ = toVec(declare_parameter<std::vector<double>>("bounds.min", {-20.0, -20.0, 0.5}));
  bounds_max_ = toVec(declare_parameter<std::vector<double>>("bounds.max", {20.0, 20.0, 4.0}));

  solve_time_ = declare_parameter<double>("planner.solve_time", 1.0);
  planner_range_ = declare_parameter<double>("planner.range", 4.0);
  k_nearest_ = declare_parameter<bool>("planner.k_nearest", false);
  objective_ = declare_parameter<std::string>("planner.objective", "path_length_cost_to_go");
  cost_threshold_ = declare_parameter<double>("planner.cost_threshold", 10.0);
  length_weight_ = declare_parameter<double>("planner.length_weight", 10.0);
  clearance_weight_ = declare_parameter<double>("planner.clearance_weight", 1.0);
  check_resolution_ = declare_parameter<double>("planner.collision_check_resolution", 0.1);
  waypoint_spacing_ = declare_parameter<double>("planner.waypoint_spacing", 1.0);
  smooth_path_ = declare_parameter<bool>("planner.smooth", true);

  const double control_rate = declare_parameter<double>("control.rate_hz", 20.0);
  const double replan_rate = declare_parameter<double>("control.replan_check_rate_hz", 2.0);
  acceptance_radius_ = declare_parameter<double>("control.acceptance_radius", 0.3);
  takeoff_altitude_ = declare_parameter<double>("control.takeoff_altitude", 2.0);
  default_goal_altitude_ = declare_parameter<double>("control.default_goal_altitude", 2.0);
  traj_limits_.max_speed = declare_parameter<double>("control.max_speed", 1.5);
  traj_limits_.max_accel = declare_parameter<double>("control.max_accel", 1.0);
  traj_limits_.max_lateral_accel = declare_parameter<double>("control.max_lateral_accel", 1.0);
  traj_limits_.max_jerk = declare_parameter<double>("control.max_jerk", 2.0);
  max_yaw_rate_ = declare_parameter<double>("control.max_yaw_rate", 1.0);
  lag_soft_ = declare_parameter<double>("control.tracking_lag_soft", 1.0);
  lag_max_ = declare_parameter<double>("control.tracking_lag_max", 2.0);
  stop_margin_ = declare_parameter<double>("control.stop_margin", 0.5);
  max_attempts_ = static_cast<int>(declare_parameter<int>("planner.max_attempts", 3));
  auto_arm_ = declare_parameter<bool>("control.auto_arm", false);
  reset_map_on_takeoff_ = declare_parameter<bool>("control.reset_map_on_takeoff", true);

  target_system_ = declare_parameter<int>("px4.target_system", 1);
  target_component_ = declare_parameter<int>("px4.target_component", 1);

  const auto t_odom = declare_parameter<std::string>(
    "topics.vehicle_odometry", "/fmu/out/vehicle_odometry");
  const auto t_status = declare_parameter<std::string>(
    "topics.vehicle_status", "/fmu/out/vehicle_status");
  const auto t_offboard = declare_parameter<std::string>(
    "topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
  const auto t_setpoint = declare_parameter<std::string>(
    "topics.trajectory_setpoint", "/fmu/in/trajectory_setpoint");
  const auto t_command = declare_parameter<std::string>(
    "topics.vehicle_command", "/fmu/in/vehicle_command");
  const auto t_cloud = declare_parameter<std::string>("topics.cloud", "/lidar/points");
  const auto t_goal = declare_parameter<std::string>("topics.goal", "/goal_pose");

  // ---------------- interfaces ----------------
  const auto px4_qos = rclcpp::SensorDataQoS().keep_last(5);
  odom_sub_ = create_subscription<VehicleOdometry>(
    t_odom, px4_qos, std::bind(&InformedRRTPlanner::odometryCallback, this, std::placeholders::_1));
  status_sub_ = create_subscription<VehicleStatus>(
    t_status, px4_qos, std::bind(&InformedRRTPlanner::statusCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    t_goal, rclcpp::QoS(1),
    std::bind(&InformedRRTPlanner::goalCallback, this, std::placeholders::_1));

  offboard_pub_ = create_publisher<OffboardControlMode>(t_offboard, 10);
  setpoint_pub_ = create_publisher<TrajectorySetpoint>(t_setpoint, 10);
  command_pub_ = create_publisher<VehicleCommand>(t_command, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/path_markers", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "~/planned_path", rclcpp::QoS(1).transient_local());
  map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/occupied_voxels", rclcpp::QoS(1).transient_local());
  reset_map_srv_ = create_service<std_srvs::srv::Empty>(
    "~/reset_map",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {resetMap();});

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Scans are held back until map_frame -> cloud frame is known at their stamp.
  cloud_sub_.subscribe(this, t_cloud, rclcpp::SensorDataQoS());
  cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    cloud_sub_, *tf_buffer_, map_frame_, 5, get_node_logging_interface(),
    get_node_clock_interface(), 1s);
  cloud_filter_->registerCallback(&InformedRRTPlanner::cloudCallback, this);

  updateWorlds();

  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  planner_thread_ = std::thread(&InformedRRTPlanner::plannerWorker, this);

  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate), [this]() {controlLoop();});
  replan_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / replan_rate), [this]() {replanCheck();});

  RCLCPP_INFO(
    get_logger(), "Informed-RRT* planner ready (objective: %s, frame: %s, auto_arm: %s)",
    objective_.c_str(), world_frame_ == WorldFrame::ENU ? "ENU" : "NWU",
    auto_arm_ ? "true" : "false");
}

InformedRRTPlanner::~InformedRRTPlanner()
{
  shutdown_ = true;
  plan_cv_.notify_all();
  if (planner_thread_.joinable()) {planner_thread_.join();}
}

// ============================================================================
// Callbacks
// ============================================================================

void InformedRRTPlanner::odometryCallback(const VehicleOdometry::SharedPtr msg)
{
  if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "VehicleOdometry pose_frame is not NED, ignoring");
    return;
  }
  if (!std::isfinite(msg->position[0]) || !std::isfinite(msg->position[2])) {return;}
  position_ = nedToWorld({msg->position[0], msg->position[1], msg->position[2]}, world_frame_);
  yaw_ = nedYawToWorld(yawFromPx4Quaternion(msg->q.data()), world_frame_);
  odom_received_ = true;
}

void InformedRRTPlanner::statusCallback(const VehicleStatus::SharedPtr msg)
{
  nav_state_ = msg->nav_state;
  arming_state_ = msg->arming_state;
  status_received_ = true;
}

void InformedRRTPlanner::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping scan: %s", e.what());
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();

  const auto & tr = tf.transform.translation;
  const auto & q = tf.transform.rotation;
  const Eigen::Matrix3d rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  const Eigen::Vector3d origin(tr.x, tr.y, tr.z);

  cloud_points_.clear();
  cloud_points_.reserve(static_cast<size_t>(msg->width) * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    cloud_points_.push_back(rot * Eigen::Vector3d(*x, *y, *z) + origin);
  }
  map_->insert(cloud_points_, origin);
  const auto t1 = std::chrono::steady_clock::now();
  updateWorlds();
  const auto t2 = std::chrono::steady_clock::now();

  insert_ms_sum_ += std::chrono::duration<double, std::milli>(t1 - t0).count();
  snapshot_ms_sum_ += std::chrono::duration<double, std::milli>(t2 - t1).count();
  ++scans_since_log_;
  ++scans_total_;
  if (scans_since_log_ >= 100) {
    const double n = static_cast<double>(scans_since_log_);
    RCLCPP_INFO(
      get_logger(), "Map: %zu scans, %zu occupied voxels, %.2f MB; per scan %.2f ms insert + "
      "%.2f ms snapshot", scans_total_, last_snapshot_size_,
      static_cast<double>(map_->memoryUsage()) / (1024.0 * 1024.0),
      insert_ms_sum_ / n, snapshot_ms_sum_ / n);
    insert_ms_sum_ = snapshot_ms_sum_ = 0.0;
    scans_since_log_ = 0;
  }
}

void InformedRRTPlanner::updateWorlds()
{
  auto snap = map_->snapshot();
  last_snapshot_size_ = snap->size();
  auto world = std::make_shared<const CollisionWorld>(snap, robot_size_);
  auto plan_world = std::make_shared<const CollisionWorld>(snap, robot_plan_size_);
  {
    std::lock_guard<std::mutex> lk(world_mutex_);
    world_ = std::move(world);
    plan_world_ = std::move(plan_world);
  }
  const auto now = this->now();
  if (map_publish_period_ > 0.0 && map_pub_->get_subscription_count() > 0 &&
    (!last_map_publish_ || (now - *last_map_publish_).seconds() >= map_publish_period_))
  {
    last_map_publish_ = now;
    publishOccupiedVoxels(*snap);
  }
}

void InformedRRTPlanner::publishOccupiedVoxels(const OccupancySnapshot & snap)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_;
  m.header.stamp = now();
  m.ns = "occupied";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  const double res = snap.resolution();
  m.scale.x = m.scale.y = m.scale.z = res;
  m.points.reserve(snap.size());
  m.colors.reserve(snap.size());
  // Colour by height, like octomap_server's height map: blue (low) -> red (high).
  const double z_lo = map_->options().min_z, z_hi = bounds_max_.z() + 1.0;
  for (const auto & c : snap.voxels()) {
    geometry_msgs::msg::Point p;
    p.x = (c.x + 0.5) * res;
    p.y = (c.y + 0.5) * res;
    p.z = (c.z + 0.5) * res;
    m.points.push_back(p);
    const double h = std::clamp((p.z - z_lo) / std::max(z_hi - z_lo, 1e-3), 0.0, 1.0);
    std_msgs::msg::ColorRGBA col;
    col.r = static_cast<float>(std::clamp(2.0 * h - 0.5, 0.0, 1.0));
    col.g = static_cast<float>(1.0 - std::abs(2.0 * h - 1.0));
    col.b = static_cast<float>(std::clamp(1.5 - 2.0 * h, 0.0, 1.0));
    col.a = 1.0f;
    m.colors.push_back(col);
  }
  if (m.points.empty()) {m.action = visualization_msgs::msg::Marker::DELETE;}
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(std::move(m));
  map_pub_->publish(arr);
}

void InformedRRTPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped goal = *msg;
  if (!goal.header.frame_id.empty() && goal.header.frame_id != map_frame_) {
    try {
      goal = tf_buffer_->transform(goal, map_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "Cannot transform goal into '%s': %s", map_frame_.c_str(), e.what());
      return;
    }
  }
  Eigen::Vector3d g(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  // RViz "2D Goal Pose" has z = 0: replace it with the configured altitude
  // (the ROS 1 version asked for it on stdin).
  if (g.z() < bounds_min_.z()) {g.z() = default_goal_altitude_;}
  g = g.cwiseMax(bounds_min_).cwiseMin(bounds_max_);

  goal_ = g;
  RCLCPP_INFO(get_logger(), "New goal: [%.2f, %.2f, %.2f]", g.x(), g.y(), g.z());

  if (state_ == FlightState::FOLLOW_PATH && !traj_.empty()) {
    // Keep flying the current path while the new one is planned, then splice onto it.
    brake_s_.reset();
    replanWhileMoving(std::nullopt);
  } else if (state_ == FlightState::HOVER || state_ == FlightState::PLANNING) {
    state_ = FlightState::PLANNING;
    requestPlan(hold_position_, g);
  } else {
    goal_pending_ = true;
    RCLCPP_INFO(get_logger(), "Vehicle not airborne yet: planning will start after takeoff");
  }
}

// ============================================================================
// Control loop / state machine (runs at control.rate_hz)
// ============================================================================

void InformedRRTPlanner::controlLoop()
{
  if (!odom_received_) {return;}

  const rclcpp::Time t_now = now();
  const double dt = last_tick_ ? std::clamp((t_now - *last_tick_).seconds(), 0.0, 0.2) : 0.0;
  last_tick_ = t_now;

  // Collect planner results.
  std::optional<PlanResult> result;
  {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    const bool waiting = state_ == FlightState::PLANNING ||
      (state_ == FlightState::FOLLOW_PATH && replan_pending_);
    if (plan_result_ && plan_result_->seq == plan_seq_ && waiting) {result = std::move(plan_result_);}
    plan_result_.reset();
  }
  if (result && result->success) {
    startTrajectory(std::move(result->waypoints));
  } else if (result && goal_) {
    const bool moving = state_ == FlightState::FOLLOW_PATH;
    if (current_attempt_ < max_attempts_) {
      RCLCPP_WARN(
        get_logger(), "No solution (attempt %d/%d): retrying with %.1f s", current_attempt_,
        max_attempts_, solve_time_ * std::pow(2.0, current_attempt_));
      if (moving) {
        replanWhileMoving(brake_s_, current_attempt_ + 1);
      } else {
        requestPlan(hold_position_, *goal_, current_attempt_ + 1);
      }
    } else if (moving) {
      RCLCPP_WARN(get_logger(), "No solution found: stopping along the path, then hovering");
      const double stop = s_ref_ + v_ref_ * v_ref_ / (2.0 * traj_limits_.max_accel);
      brake_s_ = brake_s_ ? std::min(*brake_s_, stop) : stop;
      replan_pending_ = false;
      stop_and_hover_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "No solution found, hovering");
      enterHover(hold_position_);
    }
  }

  // Pilot took over (mode switch / disarm): stop commanding the path.
  if (status_received_ && state_ != FlightState::WAIT_FOR_ODOM &&
    state_ != FlightState::PRE_OFFBOARD && (!isOffboard() || !isArmed()))
  {
    RCLCPP_WARN(get_logger(), "Vehicle left offboard mode or disarmed: path aborted");
    path_.clear();
    traj_ = PathTrajectory();
    v_ref_ = 0.0;
    replan_pending_ = false;
    brake_s_.reset();
    stop_and_hover_ = false;
    ++plan_seq_;  // invalidate any running plan
    hold_position_ = position_;
    hold_yaw_ = yaw_;
    offboard_counter_ = 0;
    // Never fight the pilot: no automatic OFFBOARD/arm requests until the vehicle is disarmed.
    pilot_override_ = isArmed();
    state_ = FlightState::PRE_OFFBOARD;
  }
  if (pilot_override_ && status_received_ && !isArmed()) {pilot_override_ = false;}

  publishOffboardControlMode();  // must be streamed continuously (> 2 Hz)

  switch (state_) {
    case FlightState::WAIT_FOR_ODOM:
      hold_position_ = position_;
      hold_yaw_ = yaw_;
      offboard_counter_ = 0;
      state_ = FlightState::PRE_OFFBOARD;
      [[fallthrough]];

    case FlightState::PRE_OFFBOARD: {
      if (!isArmed()) {hold_position_ = position_; hold_yaw_ = yaw_;}
      publishSetpoint(hold_position_, hold_yaw_);
      ++offboard_counter_;
      // PX4 only accepts OFFBOARD after it has received a setpoint stream for a while.
      if (auto_arm_ && !pilot_override_ && offboard_counter_ >= 10 &&
        offboard_counter_ % 20 == 10)
      {
        engageOffboardAndArm();
      }
      if (!auto_arm_ || pilot_override_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Streaming setpoints: switch to OFFBOARD and arm from the RC/GCS (or set control.auto_arm)");
      }
      if (status_received_ && isOffboard() && isArmed()) {
        if (reset_map_on_takeoff_) {resetMap();}
        if (position_.z() > bounds_min_.z()) {
          RCLCPP_INFO(get_logger(), "Already airborne, skipping takeoff");
          enterHover(position_);
        } else {
          hold_position_ = position_ + Eigen::Vector3d(0, 0, takeoff_altitude_);
          RCLCPP_INFO(get_logger(), "Taking off to %.1f m", takeoff_altitude_);
          state_ = FlightState::TAKEOFF;
        }
      }
      break;
    }

    case FlightState::TAKEOFF:
      publishSetpoint(hold_position_, hold_yaw_);
      if ((position_ - hold_position_).norm() < acceptance_radius_) {
        RCLCPP_INFO(get_logger(), "Takeoff complete");
        enterHover(hold_position_);
      }
      break;

    case FlightState::HOVER:
    case FlightState::PLANNING:
      publishSetpoint(hold_position_, hold_yaw_);
      break;

    case FlightState::FOLLOW_PATH:
      followTrajectory(dt);
      break;
  }
}

void InformedRRTPlanner::enterHover(const Eigen::Vector3d & p)
{
  hold_position_ = p;
  v_ref_ = 0.0;
  brake_s_.reset();
  stop_and_hover_ = false;
  replan_pending_ = false;
  state_ = FlightState::HOVER;
  if (goal_pending_ && goal_) {
    goal_pending_ = false;
    state_ = FlightState::PLANNING;
    requestPlan(position_, *goal_);
  }
}

// Checks the remaining part of the path against the latest map; replans if it is blocked.
void InformedRRTPlanner::replanCheck()
{
  if (state_ != FlightState::FOLLOW_PATH || traj_.empty() || !goal_ || replan_pending_ ||
    stop_and_hover_)
  {
    return;
  }
  // Sample the rest of the path from the reference. Right after a replan next to a wall the
  // reference can lie inside the inflated robot box while the path leads out of it: up to
  // stop_margin of such leading in-collision path is tolerated. A block is a collision after
  // the path has been free, or a leading collision that does not end within stop_margin.
  const auto w = world();
  const double step = std::max(check_resolution_, 0.05);
  std::optional<double> s_block;
  bool seen_free = false;
  for (double s = s_ref_; s <= traj_.length() + 1e-9; s += step) {
    if (w->isFree(traj_.position(s))) {
      seen_free = true;
    } else if (seen_free || s - s_ref_ > stop_margin_) {
      s_block = seen_free ? s : s_ref_;
      break;
    }
  }
  if (s_block) {
    RCLCPP_WARN(
      get_logger(), "Path blocked %.1f m ahead: REPLANNING", std::max(0.0, *s_block - s_ref_));
    replanWhileMoving(std::max(s_ref_, *s_block - stop_margin_));
  }
}

void InformedRRTPlanner::replanWhileMoving(std::optional<double> brake_s, int attempt)
{
  if (!goal_) {return;}
  if (brake_s) {brake_s_ = brake_s_ ? std::min(*brake_s_, *brake_s) : *brake_s;}
  replan_pending_ = true;
  // Plan from where the reference will be when the plan arrives (it keeps moving meanwhile),
  // but not beyond the point where it has to stop.
  const double s_stop = brake_s_ ? *brake_s_ : traj_.length();
  const double t_plan = solve_time_ * std::pow(2.0, attempt - 1) + 0.2;
  replan_from_s_ = std::clamp(s_ref_ + v_ref_ * t_plan, s_ref_, std::max(s_ref_, s_stop));
  requestPlan(traj_.position(replan_from_s_), *goal_, attempt);
}

void InformedRRTPlanner::startTrajectory(std::vector<Eigen::Vector3d> waypoints)
{
  std::vector<Eigen::Vector3d> pts;
  if (state_ == FlightState::FOLLOW_PATH && !traj_.empty()) {
    // Splice: keep the reference where it is and continue along the old (free) path to the
    // replan start, then the new path. If the reference already passed the replan start,
    // join the new path at the point closest to the reference instead.
    const Eigen::Vector3d ref = traj_.position(s_ref_);
    if (s_ref_ <= replan_from_s_ + 1e-6) {
      pts = traj_.slice(s_ref_, replan_from_s_);
      pts.insert(pts.end(), waypoints.begin() + (waypoints.size() > 1 ? 1 : 0), waypoints.end());
    } else {
      const PathTrajectory fresh(waypoints, traj_limits_);
      pts.push_back(ref);
      const auto rest = fresh.slice(fresh.project(ref), fresh.length());
      pts.insert(pts.end(), rest.begin(), rest.end());
    }
  } else {
    // From hover: start at the hold point so the reference does not jump.
    pts.push_back(hold_position_);
    pts.insert(pts.end(), waypoints.begin(), waypoints.end());
    v_ref_ = 0.0;
    yaw_ref_ = hold_yaw_;
  }
  traj_ = PathTrajectory(std::move(pts), traj_limits_);
  s_ref_ = 0.0;
  replan_pending_ = false;
  brake_s_.reset();
  stop_and_hover_ = false;
  if (traj_.empty()) {
    RCLCPP_INFO(get_logger(), "GOAL REACHED");
    goal_.reset();
    enterHover(hold_position_);
    return;
  }
  path_ = traj_.points();
  wp_idx_ = 1;
  state_ = FlightState::FOLLOW_PATH;
  RCLCPP_INFO(
    get_logger(), "Following path: %.1f m, %zu points, entry speed %.2f m/s", traj_.length(),
    path_.size(), v_ref_);
}

// Moves the reference along the trajectory with bounded acceleration and sends PX4 position,
// velocity feed-forward and a rate-limited yaw, so the vehicle flies through the waypoints
// instead of stopping at each of them.
void InformedRRTPlanner::followTrajectory(double dt)
{
  if (traj_.empty()) {
    enterHover(position_);
    return;
  }
  const double a = traj_limits_.max_accel;
  // Speed limit here and where the reference will be once a jerk-limited deceleration has
  // ramped in (a / max_jerk seconds), so braking starts early enough to be smooth.
  const double s_ahead = s_ref_ + v_ref_ * a / traj_limits_.max_jerk;
  double v_cap = std::min(traj_.speedLimit(s_ref_), traj_.speedLimit(s_ahead));
  if (brake_s_) {
    const double d = std::max(0.0, *brake_s_ - std::max(s_ref_, std::min(s_ahead, *brake_s_)));
    v_cap = std::min(v_cap, std::sqrt(2.0 * a * d));
  }
  // Do not run away from a vehicle that cannot keep up.
  const double lag = (traj_.position(s_ref_) - position_).norm();
  if (lag > lag_soft_) {v_cap *= std::clamp((lag_max_ - lag) / (lag_max_ - lag_soft_), 0.0, 1.0);}

  // Jerk-limited speed control: the acceleration needed to reach v_cap within one control
  // period (at least 0.1 s) is clamped to [-2a, a] and may change by at most max_jerk * dt
  // per tick. The braking profile uses a, so the jerk-limited ramp up to 2a has room to
  // follow it; if it falls behind, the clamp below holds the speed at v_cap.
  const double j = traj_limits_.max_jerk;
  double a_des = dt > 0.0 ? (v_cap - v_ref_) / std::max(dt, 0.1) : 0.0;
  a_des = std::clamp(a_des, -2.0 * a, a);
  // Taper the acceleration so it reaches zero as the speed reaches v_cap (v + a^2 / 2j).
  if (a_des > 0.0) {a_des = std::min(a_des, std::sqrt(2.0 * j * std::max(0.0, v_cap - v_ref_)));}
  a_ref_ = std::clamp(a_des, a_ref_ - j * dt, a_ref_ + j * dt);
  // Never exceed the speed limit, even if the jerk limit is still ramping the braking up.
  const double v_prev = v_ref_;
  v_ref_ = std::clamp(v_ref_ + a_ref_ * dt, 0.0, std::max(v_cap, 0.0));
  if (dt > 0.0) {a_ref_ = (v_ref_ - v_prev) / dt;}
  const double s_end = brake_s_ ? std::max(s_ref_, *brake_s_) : traj_.length();
  // The braking profile tends to zero at the stop point: creep the last centimetres.
  if (v_ref_ < 0.05 && s_ref_ < s_end - 1e-3 && lag < lag_soft_) {
    v_ref_ = 0.05;
    a_ref_ = 0.0;  // the creep is a constant speed: no acceleration feed-forward
  }
  s_ref_ = std::min(s_ref_ + v_ref_ * dt, s_end);
  if (s_ref_ >= s_end - 1e-6) {
    v_ref_ = std::min(v_ref_, v_cap);
    a_ref_ = 0.0;
  }

  wp_idx_ = std::min(traj_.segmentIndex(s_ref_) + 1, path_.size() - 1);
  const Eigen::Vector3d p = traj_.position(s_ref_);
  const Eigen::Vector3d t = traj_.tangent(s_ref_);
  if (t.head<2>().norm() > 0.3) {
    const double dyaw = wrapPi(std::atan2(t.y(), t.x()) - yaw_ref_);
    yaw_ref_ = wrapPi(yaw_ref_ + std::clamp(dyaw, -max_yaw_rate_ * dt, max_yaw_rate_ * dt));
  }
  const bool stopped_at_end = s_ref_ >= s_end - 1e-6;
  publishSetpoint(
    p, yaw_ref_, stopped_at_end ? Eigen::Vector3d::Zero() : Eigen::Vector3d(v_ref_ * t),
    stopped_at_end ? Eigen::Vector3d::Zero() : Eigen::Vector3d(a_ref_ * t));
  hold_position_ = p;
  hold_yaw_ = yaw_ref_;

  if (s_ref_ >= traj_.length() - 1e-6 && !replan_pending_ &&
    (position_ - traj_.end()).norm() < acceptance_radius_)
  {
    RCLCPP_INFO(get_logger(), "GOAL REACHED");
    path_.clear();
    goal_.reset();
    enterHover(traj_.end());
  } else if (stop_and_hover_ && stopped_at_end) {
    enterHover(p);
  }
}

// ============================================================================
// Planning (separate thread so the setpoint stream is never interrupted)
// ============================================================================

std::shared_ptr<const CollisionWorld> InformedRRTPlanner::planWorld() const
{
  std::lock_guard<std::mutex> lk(world_mutex_);
  return plan_world_;
}

std::shared_ptr<const CollisionWorld> InformedRRTPlanner::world() const
{
  std::lock_guard<std::mutex> lk(world_mutex_);
  return world_;
}

void InformedRRTPlanner::requestPlan(
  const Eigen::Vector3d & start, const Eigen::Vector3d & goal, int attempt)
{
  std::lock_guard<std::mutex> lk(plan_mutex_);
  current_attempt_ = attempt;
  plan_request_ = PlanRequest{
    start, goal, ++plan_seq_, solve_time_ * std::pow(2.0, attempt - 1), attempt};
  plan_result_.reset();
  plan_cv_.notify_one();
}

void InformedRRTPlanner::plannerWorker()
{
  while (!shutdown_) {
    PlanRequest req;
    {
      std::unique_lock<std::mutex> lk(plan_mutex_);
      plan_cv_.wait(lk, [this]() {return shutdown_ || plan_request_.has_value();});
      if (shutdown_) {return;}
      req = *plan_request_;
      plan_request_.reset();
    }
    PlanResult res;
    try {
      res = solve(req, planWorld());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Planner exception: %s", e.what());
      res = PlanResult{false, {}, req.seq};
    }
    if (res.success) {publishPath(res.waypoints);}
    std::lock_guard<std::mutex> lk(plan_mutex_);
    if (res.seq == plan_seq_) {plan_result_ = std::move(res);}
  }
}

InformedRRTPlanner::PlanResult InformedRRTPlanner::solve(
  const PlanRequest & req, std::shared_ptr<const CollisionWorld> w)
{
  PlanResult out{false, {}, req.seq};
  if (!w->hasMap()) {
    RCLCPP_WARN(get_logger(), "Map is empty (no lidar scan yet): planning in free space");
  }

  auto space = std::make_shared<ob::RealVectorStateSpace>(3);
  ob::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; ++i) {
    bounds.setLow(i, bounds_min_[i]);
    bounds.setHigh(i, bounds_max_[i]);
  }
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  // NaN fails no bounds comparison, so satisfiesBounds() accepts it: count such states.
  auto non_finite = std::make_shared<std::atomic<size_t>>(0);
  si->setStateValidityChecker(
    [w, si_raw = si.get(), non_finite](const ob::State * s) {
      const Eigen::Vector3d p = toVec(s);
      if (!p.allFinite()) {++*non_finite; return false;}
      return si_raw->satisfiesBounds(s) && w->isFree(p);
    });
  si->setStateValidityCheckingResolution(check_resolution_ / space->getMaximumExtent());
  si->setup();

  ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
  for (int i = 0; i < 3; ++i) {
    start[i] = std::clamp(req.start[i], bounds_min_[i], bounds_max_[i]);
    goal[i] = req.goal[i];
  }

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start, goal, acceptance_radius_);
  // Start may lie inside an inflated voxel (e.g. close to a wall): nudge it out.
  pdef->fixInvalidInputStates(1.2, 1.2, 100);

  // Optimisation objective (was read from stdin in the ROS 1 version).
  ob::OptimizationObjectivePtr obj;
  bool informed = true;
  if (objective_ == "path_length") {
    obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
  } else if (objective_ == "path_length_cost_to_go") {
    obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
  } else if (objective_ == "threshold") {
    obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    obj->setCostThreshold(ob::Cost(cost_threshold_));
  } else if (objective_ == "balanced") {
    auto multi = std::make_shared<ob::MultiOptimizationObjective>(si);
    multi->addObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si), length_weight_);
    multi->addObjective(std::make_shared<ClearanceObjective>(si, w), clearance_weight_);
    obj = multi;
    informed = false;  // informed sampling is only defined for path length objectives
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown planner.objective '%s'", objective_.c_str());
    return out;
  }
  pdef->setOptimizationObjective(obj);

  ob::PlannerPtr planner;
  if (informed) {
    auto p = std::make_shared<og::InformedRRTstar>(si);
    p->setRange(planner_range_);
    p->setKNearest(k_nearest_);
    // OMPL 1.6's default NearestNeighborsGNAT returns duplicate motions from list() after
    // the start state is moved by fixInvalidInputStates(); RRTstar::freeMemory then frees
    // the same motion twice and segfaults. SqrtApprox has no such bug.
    p->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
    planner = p;
  } else {
    auto p = std::make_shared<og::RRTstar>(si);
    p->setRange(planner_range_);
    p->setKNearest(k_nearest_);
    p->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();  // see InformedRRTstar above
    planner = p;
  }
  planner->setProblemDefinition(pdef);
  planner->setup();

  // Abort early if a newer request supersedes this one.
  const uint64_t seq = req.seq;
  auto ptc = ob::plannerOrTerminationCondition(
    ob::timedPlannerTerminationCondition(req.solve_time),
    ob::PlannerTerminationCondition([this, seq]() {return shutdown_ || plan_seq_ != seq;}));

  const auto t0 = std::chrono::steady_clock::now();
  const ob::PlannerStatus status = planner->solve(ptc);
  if (*non_finite > 0) {
    RCLCPP_WARN(get_logger(), "OMPL produced %zu non-finite states (rejected)", non_finite->load());
  }
  if (status != ob::PlannerStatus::EXACT_SOLUTION) {
    RCLCPP_WARN(get_logger(), "Planner status: %s", status.asString().c_str());
    return out;
  }

  auto path = *pdef->getSolutionPath()->as<og::PathGeometric>();
  if (smooth_path_) {
    og::PathSimplifier simplifier(si);
    og::PathGeometric smoothed = path;
    simplifier.reduceVertices(smoothed);
    simplifier.smoothBSpline(smoothed, 2);
    if (smoothed.check()) {path = smoothed;}
  }
  if (waypoint_spacing_ > 0.0) {
    path.interpolate(
      static_cast<unsigned int>(std::ceil(path.length() / waypoint_spacing_)) + 1);
  }

  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    out.waypoints.push_back(toVec(path.getState(i)));
  }
  out.success = true;
  RCLCPP_INFO(
    get_logger(), "Found path: %zu waypoints, %.2f m, %.0f ms", out.waypoints.size(),
    path.length(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  return out;
}

void InformedRRTPlanner::publishPath(const std::vector<Eigen::Vector3d> & wps)
{
  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = stamp;

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker spheres, line;
  spheres.header = path.header;
  spheres.ns = "waypoints";
  spheres.id = 0;
  spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::msg::Marker::ADD;
  spheres.pose.orientation.w = 1.0;
  spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.25;
  spheres.color.g = 0.8f;
  spheres.color.a = 0.8f;
  line = spheres;
  line.ns = "path";
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = 0.05;
  line.color.r = 0.2f;

  for (const auto & p : wps) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
    spheres.points.push_back(ps.pose.position);
  }
  line.points = spheres.points;
  markers.markers.push_back(spheres);
  markers.markers.push_back(line);
  marker_pub_->publish(markers);
  path_pub_->publish(path);
}

// ============================================================================
// PX4 helpers
// ============================================================================

uint64_t InformedRRTPlanner::px4Timestamp() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

bool InformedRRTPlanner::isOffboard() const
{
  return nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool InformedRRTPlanner::isArmed() const
{
  return arming_state_ == VehicleStatus::ARMING_STATE_ARMED;
}

void InformedRRTPlanner::publishOffboardControlMode()
{
  OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = px4Timestamp();
  offboard_pub_->publish(msg);
}

void InformedRRTPlanner::publishSetpoint(
  const Eigen::Vector3d & pos_world, double yaw_world,
  const std::optional<Eigen::Vector3d> & vel_world,
  const std::optional<Eigen::Vector3d> & acc_world)
{
  const Eigen::Vector3d ned = worldToNed(pos_world, world_frame_);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  TrajectorySetpoint msg{};
  msg.position = {static_cast<float>(ned.x()), static_cast<float>(ned.y()),
    static_cast<float>(ned.z())};
  msg.velocity = {nan, nan, nan};
  if (vel_world) {
    // worldToNed only swaps and negates axes, so it maps velocities as well.
    const Eigen::Vector3d v = worldToNed(*vel_world, world_frame_);
    msg.velocity = {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
  }
  msg.acceleration = {nan, nan, nan};
  if (acc_world) {
    const Eigen::Vector3d acc = worldToNed(*acc_world, world_frame_);
    msg.acceleration = {
      static_cast<float>(acc.x()), static_cast<float>(acc.y()), static_cast<float>(acc.z())};
  }
  msg.jerk = {nan, nan, nan};
  msg.yaw = static_cast<float>(worldYawToNed(yaw_world, world_frame_));
  msg.yawspeed = nan;
  msg.timestamp = px4Timestamp();
  setpoint_pub_->publish(msg);
}

void InformedRRTPlanner::publishVehicleCommand(uint32_t command, float p1, float p2)
{
  VehicleCommand msg{};
  msg.param1 = p1;
  msg.param2 = p2;
  msg.command = command;
  msg.target_system = static_cast<uint8_t>(target_system_);
  msg.target_component = static_cast<uint8_t>(target_component_);
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = px4Timestamp();
  command_pub_->publish(msg);
}

void InformedRRTPlanner::engageOffboardAndArm()
{
  if (!isOffboard()) {
    // param1 = 1 (custom mode), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
    RCLCPP_INFO(get_logger(), "Requesting OFFBOARD mode");
  }
  if (!isArmed()) {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
    RCLCPP_INFO(get_logger(), "Requesting ARM");
  }
}

void InformedRRTPlanner::resetMap()
{
  map_->reset();
  last_map_publish_.reset();
  updateWorlds();
  RCLCPP_INFO(get_logger(), "Map reset");
}

}  // namespace path_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(path_planning::InformedRRTPlanner)
