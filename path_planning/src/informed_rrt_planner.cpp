#include "path_planning/informed_rrt_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <octomap_msgs/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  robot_geom_ = std::make_shared<fcl::Boxd>(robot_size.x(), robot_size.y(), robot_size.z());

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
  yaw_min_dist_ = declare_parameter<double>("control.yaw_min_distance", 0.3);
  auto_arm_ = declare_parameter<bool>("control.auto_arm", false);
  reset_octomap_on_takeoff_ = declare_parameter<bool>("control.reset_octomap_on_takeoff", true);

  target_system_ = declare_parameter<int>("px4.target_system", 1);
  target_component_ = declare_parameter<int>("px4.target_component", 1);

  const auto t_odom = declare_parameter<std::string>(
    "topics.vehicle_odometry", "/fmu/out/vehicle_odometry");
  const auto t_status = declare_parameter<std::string>(
    "topics.vehicle_status", "/fmu/out/vehicle_status_v5");
  const auto t_offboard = declare_parameter<std::string>(
    "topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
  const auto t_setpoint = declare_parameter<std::string>(
    "topics.trajectory_setpoint", "/fmu/in/trajectory_setpoint");
  const auto t_command = declare_parameter<std::string>(
    "topics.vehicle_command", "/fmu/in/vehicle_command_v1");
  const auto t_octomap = declare_parameter<std::string>("topics.octomap", "/octomap_binary");
  const auto t_goal = declare_parameter<std::string>("topics.goal", "/goal_pose");
  const auto s_reset = declare_parameter<std::string>(
    "topics.octomap_reset_service", "/octomap_server/reset");

  // ---------------- interfaces ----------------
  const auto px4_qos = rclcpp::SensorDataQoS().keep_last(5);
  odom_sub_ = create_subscription<VehicleOdometry>(
    t_odom, px4_qos, std::bind(&InformedRRTPlanner::odometryCallback, this, std::placeholders::_1));
  status_sub_ = create_subscription<VehicleStatus>(
    t_status, px4_qos, std::bind(&InformedRRTPlanner::statusCallback, this, std::placeholders::_1));
  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    t_octomap, rclcpp::QoS(1),
    std::bind(&InformedRRTPlanner::octomapCallback, this, std::placeholders::_1));
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
  octomap_reset_client_ = create_client<std_srvs::srv::Empty>(s_reset);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  {
    std::lock_guard<std::mutex> lk(world_mutex_);
    world_ = std::make_shared<const CollisionWorld>(nullptr, robot_geom_);
  }

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

void InformedRRTPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Octomap frame '%s' differs from map_frame '%s'",
      msg->header.frame_id.c_str(), map_frame_.c_str());
  }
  std::unique_ptr<octomap::AbstractOcTree> abstract(
    msg->binary ? octomap_msgs::binaryMsgToMap(*msg) : octomap_msgs::fullMsgToMap(*msg));
  auto * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
  if (!tree) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Received octomap is not an OcTree");
    return;
  }
  abstract.release();
  auto world = std::make_shared<const CollisionWorld>(
    std::shared_ptr<const octomap::OcTree>(tree), robot_geom_);
  std::lock_guard<std::mutex> lk(world_mutex_);
  world_ = std::move(world);
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

  if (state_ == FlightState::HOVER || state_ == FlightState::FOLLOW_PATH ||
    state_ == FlightState::PLANNING)
  {
    hold_position_ = position_;
    hold_yaw_ = yaw_;
    state_ = FlightState::PLANNING;
    requestPlan(position_, g);
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

  // Collect planner results.
  {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    if (plan_result_ && plan_result_->seq == plan_seq_ && state_ == FlightState::PLANNING) {
      if (plan_result_->success) {
        path_ = std::move(plan_result_->waypoints);
        wp_idx_ = path_.size() > 1 ? 1 : 0;
        state_ = FlightState::FOLLOW_PATH;
        RCLCPP_INFO(get_logger(), "Following path with %zu waypoints", path_.size());
      } else {
        RCLCPP_WARN(get_logger(), "No solution found, hovering");
        enterHover(hold_position_);
      }
    }
    plan_result_.reset();
  }

  // Pilot took over (mode switch / disarm): stop commanding the path.
  if (status_received_ && state_ != FlightState::WAIT_FOR_ODOM &&
    state_ != FlightState::PRE_OFFBOARD && (!isOffboard() || !isArmed()))
  {
    RCLCPP_WARN(get_logger(), "Vehicle left offboard mode or disarmed: path aborted");
    path_.clear();
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
        if (reset_octomap_on_takeoff_) {resetOctomap();}
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

    case FlightState::FOLLOW_PATH: {
      const Eigen::Vector3d & wp = path_[wp_idx_];
      const Eigen::Vector3d d = wp - position_;
      if (d.head<2>().norm() > yaw_min_dist_) {hold_yaw_ = std::atan2(d.y(), d.x());}
      publishSetpoint(wp, hold_yaw_);
      if (d.norm() < acceptance_radius_) {
        if (++wp_idx_ >= path_.size()) {
          RCLCPP_INFO(get_logger(), "GOAL REACHED");
          path_.clear();
          goal_.reset();
          enterHover(wp);
        }
      }
      break;
    }
  }
}

void InformedRRTPlanner::enterHover(const Eigen::Vector3d & p)
{
  hold_position_ = p;
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
  if (state_ != FlightState::FOLLOW_PATH || path_.empty() || !goal_) {return;}
  const auto w = world();
  bool blocked = !w->isSegmentFree(position_, path_[wp_idx_], check_resolution_);
  for (size_t i = wp_idx_; !blocked && i + 1 < path_.size(); ++i) {
    blocked = !w->isSegmentFree(path_[i], path_[i + 1], check_resolution_);
  }
  if (blocked) {
    RCLCPP_WARN(get_logger(), "Path blocked by new obstacle: REPLANNING");
    hold_position_ = position_;
    state_ = FlightState::PLANNING;
    requestPlan(position_, *goal_);
  }
}

// ============================================================================
// Planning (separate thread so the setpoint stream is never interrupted)
// ============================================================================

std::shared_ptr<const CollisionWorld> InformedRRTPlanner::world() const
{
  std::lock_guard<std::mutex> lk(world_mutex_);
  return world_;
}

void InformedRRTPlanner::requestPlan(const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  std::lock_guard<std::mutex> lk(plan_mutex_);
  plan_request_ = PlanRequest{start, goal, ++plan_seq_};
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
      res = solve(req, world());
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
    RCLCPP_WARN(get_logger(), "No octomap received yet: planning in free space");
  }

  auto space = std::make_shared<ob::RealVectorStateSpace>(3);
  ob::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; ++i) {
    bounds.setLow(i, bounds_min_[i]);
    bounds.setHigh(i, bounds_max_[i]);
  }
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(
    [w, si_raw = si.get()](const ob::State * s) {
      return si_raw->satisfiesBounds(s) && w->isFree(toVec(s));
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
    planner = p;
  } else {
    auto p = std::make_shared<og::RRTstar>(si);
    p->setRange(planner_range_);
    p->setKNearest(k_nearest_);
    planner = p;
  }
  planner->setProblemDefinition(pdef);
  planner->setup();

  // Abort early if a newer request supersedes this one.
  const uint64_t seq = req.seq;
  auto ptc = ob::plannerOrTerminationCondition(
    ob::timedPlannerTerminationCondition(solve_time_),
    ob::PlannerTerminationCondition([this, seq]() {return shutdown_ || plan_seq_ != seq;}));

  const auto t0 = std::chrono::steady_clock::now();
  const ob::PlannerStatus status = planner->solve(ptc);
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

void InformedRRTPlanner::publishSetpoint(const Eigen::Vector3d & pos_world, double yaw_world)
{
  const Eigen::Vector3d ned = worldToNed(pos_world, world_frame_);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  TrajectorySetpoint msg{};
  msg.position = {static_cast<float>(ned.x()), static_cast<float>(ned.y()),
    static_cast<float>(ned.z())};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
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

void InformedRRTPlanner::resetOctomap()
{
  if (!octomap_reset_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Octomap reset service not available, keeping current map");
    return;
  }
  octomap_reset_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  RCLCPP_INFO(get_logger(), "Resetting the initial octomap");
}

}  // namespace path_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(path_planning::InformedRRTPlanner)
