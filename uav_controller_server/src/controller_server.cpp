#include "uav_controller_server/controller_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;

namespace uav_controller_server
{

namespace
{

Eigen::Vector3d toVec(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {throw std::invalid_argument(name + " must have 3 elements");}
  return {v[0], v[1], v[2]};
}

}  // namespace

ControllerServer::ControllerServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("controller_server", options)
{
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("frame_convention", "nwu");
  declare_parameter<std::vector<double>>("robot.size", {0.90, 0.90, 0.60});
  declare_parameter<std::vector<double>>("bounds.min", {-20.0, -20.0, 0.5});
  declare_parameter<double>("rate_hz", 20.0);
  declare_parameter<double>("block_check_rate_hz", 2.0);
  declare_parameter<double>("collision_check_resolution", 0.1);
  declare_parameter<double>("acceptance_radius", 0.3);
  declare_parameter<double>("takeoff_altitude", 2.0);
  declare_parameter<double>("max_speed", 1.5);
  declare_parameter<double>("max_accel", 1.0);
  declare_parameter<double>("max_lateral_accel", 1.0);
  declare_parameter<double>("max_jerk", 2.0);
  declare_parameter<double>("max_yaw_rate", 1.0);
  declare_parameter<double>("tracking_lag_soft", 1.0);
  declare_parameter<double>("tracking_lag_max", 2.0);
  declare_parameter<double>("stop_margin", 0.5);
  declare_parameter<bool>("auto_arm", false);
  declare_parameter<bool>("reset_map_on_takeoff", true);
  declare_parameter<std::string>("map_reset_service", "/map_server/reset_map");
  declare_parameter<int>("px4.target_system", 1);
  declare_parameter<int>("px4.target_component", 1);
  declare_parameter<std::string>("topics.vehicle_odometry", "/fmu/out/vehicle_odometry");
  declare_parameter<std::string>("topics.vehicle_status", "/fmu/out/vehicle_status");
  declare_parameter<std::string>("topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
  declare_parameter<std::string>("topics.trajectory_setpoint", "/fmu/in/trajectory_setpoint");
  declare_parameter<std::string>("topics.vehicle_command", "/fmu/in/vehicle_command");
  declare_parameter<std::string>("topics.map", "voxel_map");
}

// ============================================================================
// Lifecycle
// ============================================================================

ControllerServer::CallbackReturn ControllerServer::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    map_frame_ = get_parameter("map_frame").as_string();
    world_frame_ = worldFrameFromString(get_parameter("frame_convention").as_string());
    robot_size_ = toVec(get_parameter("robot.size").as_double_array(), "robot.size");
    min_altitude_ = toVec(get_parameter("bounds.min").as_double_array(), "bounds.min").z();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  control_rate_ = get_parameter("rate_hz").as_double();
  block_check_rate_ = get_parameter("block_check_rate_hz").as_double();
  check_resolution_ = get_parameter("collision_check_resolution").as_double();
  acceptance_radius_ = get_parameter("acceptance_radius").as_double();
  takeoff_altitude_ = get_parameter("takeoff_altitude").as_double();
  traj_limits_.max_speed = get_parameter("max_speed").as_double();
  traj_limits_.max_accel = get_parameter("max_accel").as_double();
  traj_limits_.max_lateral_accel = get_parameter("max_lateral_accel").as_double();
  traj_limits_.max_jerk = get_parameter("max_jerk").as_double();
  max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
  lag_soft_ = get_parameter("tracking_lag_soft").as_double();
  lag_max_ = get_parameter("tracking_lag_max").as_double();
  stop_margin_ = get_parameter("stop_margin").as_double();
  auto_arm_ = get_parameter("auto_arm").as_bool();
  reset_map_on_takeoff_ = get_parameter("reset_map_on_takeoff").as_bool();
  reset_service_ = get_parameter("map_reset_service").as_string();
  target_system_ = static_cast<int>(get_parameter("px4.target_system").as_int());
  target_component_ = static_cast<int>(get_parameter("px4.target_component").as_int());
  t_odom_ = get_parameter("topics.vehicle_odometry").as_string();
  t_status_ = get_parameter("topics.vehicle_status").as_string();
  t_offboard_ = get_parameter("topics.offboard_control_mode").as_string();
  t_setpoint_ = get_parameter("topics.trajectory_setpoint").as_string();
  t_command_ = get_parameter("topics.vehicle_command").as_string();
  t_map_ = get_parameter("topics.map").as_string();
  if (!(control_rate_ > 2.0) || !(block_check_rate_ > 0.0)) {
    RCLCPP_ERROR(get_logger(), "rate_hz must be > 2 (PX4 offboard) and block_check_rate_hz > 0");
    return CallbackReturn::FAILURE;
  }

  offboard_pub_ = create_publisher<OffboardControlMode>(t_offboard_, 10);
  setpoint_pub_ = create_publisher<TrajectorySetpoint>(t_setpoint_, 10);
  command_pub_ = create_publisher<VehicleCommand>(t_command_, 10);
  state_pub_ = create_publisher<State>("~/state", rclcpp::QoS(1).transient_local());
  traj_pub_ = create_publisher<nav_msgs::msg::Path>("~/trajectory", rclcpp::QoS(1).transient_local());
  reset_client_ = create_client<std_srvs::srv::Empty>(reset_service_);
  world_ = std::make_shared<const uav_nav_core::CollisionWorld>(nullptr, robot_size_);

  RCLCPP_INFO(
    get_logger(), "Configured: %.0f Hz, frame %s, auto_arm %s, %.1f m/s max", control_rate_,
    world_frame_ == WorldFrame::ENU ? "ENU" : "NWU", auto_arm_ ? "true" : "false",
    traj_limits_.max_speed);
  return CallbackReturn::SUCCESS;
}

ControllerServer::CallbackReturn ControllerServer::on_activate(const rclcpp_lifecycle::State &)
{
  offboard_pub_->on_activate();
  setpoint_pub_->on_activate();
  command_pub_->on_activate();
  state_pub_->on_activate();
  traj_pub_->on_activate();

  const auto px4_qos = rclcpp::SensorDataQoS().keep_last(5);
  odom_sub_ = create_subscription<VehicleOdometry>(
    t_odom_, px4_qos, [this](VehicleOdometry::SharedPtr m) {odometryCallback(m);});
  status_sub_ = create_subscription<VehicleStatus>(
    t_status_, px4_qos, [this](VehicleStatus::SharedPtr m) {statusCallback(m);});
  map_sub_ = create_subscription<uav_nav_core::VoxelMap>(
    t_map_, uav_nav_core::voxelMapQoS(),
    [this](std::shared_ptr<const uav_nav_core::VoxelMap> m) {mapCallback(std::move(m));});

  action_server_ = rclcpp_action::create_server<FollowPath>(
    this, "follow_path",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<GoalHandle> gh) {acceptPath(gh);});
  replan_srv_ = create_service<uav_nav_msgs::srv::GetReplanStart>(
    "~/get_replan_start",
    [this](const std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Request> req,
    std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Response> res) {replanStart(req, res);});

  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_), [this]() {controlLoop();});
  block_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / block_check_rate_), [this]() {blockCheck();});
  return CallbackReturn::SUCCESS;
}

ControllerServer::CallbackReturn ControllerServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(get_logger(), "Deactivated: offboard setpoints stop, PX4 will fail over");
  finishGoal(false, FollowPath::Result::CANCELED, "controller server deactivated");
  control_timer_.reset();
  block_timer_.reset();
  replan_srv_.reset();
  action_server_.reset();
  map_sub_.reset();
  status_sub_.reset();
  odom_sub_.reset();
  offboard_pub_->on_deactivate();
  setpoint_pub_->on_deactivate();
  command_pub_->on_deactivate();
  state_pub_->on_deactivate();
  traj_pub_->on_deactivate();
  traj_ = PathTrajectory();
  ++traj_version_;
  state_ = State::WAIT_FOR_ODOM;
  odom_received_ = status_received_ = false;
  last_tick_.reset();
  return CallbackReturn::SUCCESS;
}

ControllerServer::CallbackReturn ControllerServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  offboard_pub_.reset();
  setpoint_pub_.reset();
  command_pub_.reset();
  state_pub_.reset();
  traj_pub_.reset();
  reset_client_.reset();
  world_.reset();
  return CallbackReturn::SUCCESS;
}

ControllerServer::CallbackReturn ControllerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  finishGoal(false, FollowPath::Result::CANCELED, "controller server shutting down");
  control_timer_.reset();
  block_timer_.reset();
  action_server_.reset();
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// Callbacks
// ============================================================================

void ControllerServer::odometryCallback(const VehicleOdometry::SharedPtr msg)
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

void ControllerServer::statusCallback(const VehicleStatus::SharedPtr msg)
{
  nav_state_ = msg->nav_state;
  arming_state_ = msg->arming_state;
  status_received_ = true;
}

void ControllerServer::mapCallback(std::shared_ptr<const uav_nav_core::VoxelMap> map)
{
  if (map && map->snapshot) {
    world_ = std::make_shared<const uav_nav_core::CollisionWorld>(map->snapshot, robot_size_);
  }
}

void ControllerServer::acceptPath(const std::shared_ptr<GoalHandle> & gh)
{
  const auto goal = gh->get_goal();
  const auto reject = [&gh](uint16_t code, const std::string & msg) {
      auto r = std::make_shared<FollowPath::Result>();
      r->error_code = code;
      r->error_msg = msg;
      gh->abort(r);
    };
  if (state_ != State::HOVER && state_ != State::FOLLOW_PATH) {
    reject(FollowPath::Result::NOT_AIRBORNE, "vehicle is not hovering or following a path");
    return;
  }
  if (!goal->path.header.frame_id.empty() && goal->path.header.frame_id != map_frame_) {
    reject(FollowPath::Result::INVALID_PATH, "path must be in " + map_frame_);
    return;
  }
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(goal->path.poses.size());
  for (const auto & ps : goal->path.poses) {
    const Eigen::Vector3d p(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    if (!p.allFinite()) {
      reject(FollowPath::Result::INVALID_PATH, "path has non-finite points");
      return;
    }
    pts.push_back(p);
  }
  if (pts.empty()) {
    reject(FollowPath::Result::INVALID_PATH, "empty path");
    return;
  }

  finishGoal(false, FollowPath::Result::REPLACED, "replaced by a newer path");
  goal_ = gh;
  goal_final_ = goal->final;
  if (!startTrajectory(std::move(pts))) {
    if (goal_final_) {
      RCLCPP_INFO(get_logger(), "GOAL REACHED (already there)");
      finishGoal(true, FollowPath::Result::NONE, "");
    } else {
      finishGoal(false, FollowPath::Result::INVALID_PATH, "degenerate intermediate segment");
    }
    enterHover(hold_position_);
  }
}

void ControllerServer::replanStart(
  const std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Request> req,
  std::shared_ptr<uav_nav_msgs::srv::GetReplanStart::Response> res)
{
  Eigen::Vector3d start = hold_position_;
  res->following = state_ == State::FOLLOW_PATH && !traj_.empty() && !stop_and_hover_;
  if (res->following) {
    // Plan from where the reference will be when the plan arrives (it keeps moving
    // meanwhile), but not beyond the point where it has to stop.
    double s = traj_.length();
    if (!req->from_end) {
      const double s_stop = brake_s_ ? *brake_s_ : traj_.length();
      s = std::clamp(
        s_ref_ + v_ref_ * std::max(0.0, req->lookahead_time), s_ref_, std::max(s_ref_, s_stop));
    }
    replan_from_ = std::make_pair(traj_version_, s);
    start = traj_.position(s);
  }
  res->start.x = start.x();
  res->start.y = start.y();
  res->start.z = start.z();
}

// ============================================================================
// Control loop / state machine (runs at rate_hz)
// ============================================================================

void ControllerServer::controlLoop()
{
  if (!odom_received_) {
    publishState(false);  // WAIT_FOR_ODOM, so the navigator knows the controller is up
    return;
  }

  const rclcpp::Time t_now = now();
  const double dt = last_tick_ ? std::clamp((t_now - *last_tick_).seconds(), 0.0, 0.2) : 0.0;
  last_tick_ = t_now;

  // Pilot took over (mode switch / disarm): stop commanding the path.
  if (status_received_ && state_ != State::WAIT_FOR_ODOM &&
    state_ != State::PRE_OFFBOARD && (!isOffboard() || !isArmed()))
  {
    RCLCPP_WARN(get_logger(), "Vehicle left offboard mode or disarmed: path aborted");
    finishGoal(false, FollowPath::Result::PILOT_OVERRIDE, "vehicle left OFFBOARD or disarmed");
    traj_ = PathTrajectory();
    ++traj_version_;
    v_ref_ = 0.0;
    brake_s_.reset();
    blocked_ = false;
    stop_and_hover_ = false;
    replan_from_.reset();
    hold_position_ = position_;
    hold_yaw_ = yaw_;
    offboard_counter_ = 0;
    // Never fight the pilot: no automatic OFFBOARD/arm requests until the vehicle is disarmed.
    pilot_override_ = isArmed();
    state_ = State::PRE_OFFBOARD;
  }
  if (pilot_override_ && status_received_ && !isArmed()) {pilot_override_ = false;}

  // A canceled goal: brake along the path, then hover (followTrajectory ends the goal).
  if (goal_ && goal_->is_canceling() && !stop_and_hover_) {
    if (state_ == State::FOLLOW_PATH && !traj_.empty()) {
      const double stop = s_ref_ + v_ref_ * v_ref_ / (2.0 * traj_limits_.max_accel);
      brake_s_ = brake_s_ ? std::min(*brake_s_, stop) : stop;
      stop_and_hover_ = true;
      RCLCPP_INFO(get_logger(), "Path canceled: stopping along the path, then hovering");
    } else {
      finishGoal(false, FollowPath::Result::CANCELED, "canceled", true);
    }
  }

  publishOffboardControlMode();  // must be streamed continuously (> 2 Hz)

  switch (state_) {
    case State::WAIT_FOR_ODOM:
      hold_position_ = position_;
      hold_yaw_ = yaw_;
      offboard_counter_ = 0;
      state_ = State::PRE_OFFBOARD;
      [[fallthrough]];

    case State::PRE_OFFBOARD: {
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
          "Streaming setpoints: switch to OFFBOARD and arm from the RC/GCS (or set auto_arm)");
      }
      if (status_received_ && isOffboard() && isArmed()) {
        if (reset_map_on_takeoff_) {requestMapReset();}
        if (position_.z() > min_altitude_) {
          RCLCPP_INFO(get_logger(), "Already airborne, skipping takeoff");
          enterHover(position_);
        } else {
          hold_position_ = position_ + Eigen::Vector3d(0, 0, takeoff_altitude_);
          RCLCPP_INFO(get_logger(), "Taking off to %.1f m", takeoff_altitude_);
          state_ = State::TAKEOFF;
        }
      }
      break;
    }

    case State::TAKEOFF:
      publishSetpoint(hold_position_, hold_yaw_);
      if ((position_ - hold_position_).norm() < acceptance_radius_) {
        RCLCPP_INFO(get_logger(), "Takeoff complete");
        enterHover(hold_position_);
      }
      break;

    case State::HOVER:
      publishSetpoint(hold_position_, hold_yaw_);
      break;

    case State::FOLLOW_PATH:
      followTrajectory(dt);
      break;
  }

  if (goal_ && ++feedback_counter_ % 2 == 0) {publishFeedback();}
  publishState(false);
}

void ControllerServer::enterHover(const Eigen::Vector3d & p)
{
  hold_position_ = p;
  v_ref_ = 0.0;
  a_ref_ = 0.0;
  brake_s_.reset();
  blocked_ = false;
  stop_and_hover_ = false;
  replan_from_.reset();
  state_ = State::HOVER;
  publishState(true);
}

// Checks the remaining part of the path against the latest map; brakes before a block.
void ControllerServer::blockCheck()
{
  if (state_ != State::FOLLOW_PATH || traj_.empty() || !goal_ || blocked_ || stop_and_hover_ ||
    !world_)
  {
    return;
  }
  // Sample the rest of the path from the reference. Right after a replan next to a wall the
  // reference can lie inside the inflated robot box while the path leads out of it: up to
  // stop_margin of such leading in-collision path is tolerated. A block is a collision after
  // the path has been free, or a leading collision that does not end within stop_margin.
  const double step = std::max(check_resolution_, 0.05);
  std::optional<double> s_block;
  bool seen_free = false;
  for (double s = s_ref_; s <= traj_.length() + 1e-9; s += step) {
    if (world_->isFree(traj_.position(s))) {
      seen_free = true;
    } else if (seen_free || s - s_ref_ > stop_margin_) {
      s_block = seen_free ? s : s_ref_;
      break;
    }
  }
  if (s_block) {
    const double brake = std::max(s_ref_, *s_block - stop_margin_);
    brake_s_ = brake_s_ ? std::min(*brake_s_, brake) : brake;
    blocked_ = true;
    blocked_s_ = *s_block;
    RCLCPP_WARN(
      get_logger(), "Path blocked %.1f m ahead: braking, replan needed",
      std::max(0.0, *s_block - s_ref_));
    publishFeedback();
  }
}

bool ControllerServer::startTrajectory(std::vector<Eigen::Vector3d> waypoints)
{
  std::vector<Eigen::Vector3d> pts;
  if (state_ == State::FOLLOW_PATH && !traj_.empty()) {
    // Splice: keep the reference where it is and continue along the old (free) path to the
    // replan start, then the new path. If the reference already passed the replan start (or
    // the path did not come from a get_replan_start answer on this trajectory), join the new
    // path at the point closest to the reference instead.
    const Eigen::Vector3d ref = traj_.position(s_ref_);
    const bool known = replan_from_ && replan_from_->first == traj_version_;
    if (known && s_ref_ <= replan_from_->second + 1e-6) {
      pts = traj_.slice(s_ref_, replan_from_->second);
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
    a_ref_ = 0.0;
    yaw_ref_ = hold_yaw_;
  }
  traj_ = PathTrajectory(std::move(pts), traj_limits_);
  ++traj_version_;
  s_ref_ = 0.0;
  brake_s_.reset();
  blocked_ = false;
  stop_and_hover_ = false;
  replan_from_.reset();
  if (traj_.empty()) {return false;}
  state_ = State::FOLLOW_PATH;
  publishTrajectory();
  publishState(true);
  RCLCPP_INFO(
    get_logger(), "Following path: %.1f m, %zu points, entry speed %.2f m/s%s", traj_.length(),
    traj_.points().size(), v_ref_, goal_final_ ? "" : " (more segments follow)");
  return true;
}

// Moves the reference along the trajectory with bounded acceleration and sends PX4 position,
// velocity feed-forward and a rate-limited yaw, so the vehicle flies through the waypoints
// instead of stopping at each of them.
void ControllerServer::followTrajectory(double dt)
{
  if (traj_.empty()) {
    finishGoal(false, FollowPath::Result::INVALID_PATH, "no trajectory to follow");
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

  if (goal_final_ && !stop_and_hover_ && s_ref_ >= traj_.length() - 1e-6 &&
    (position_ - traj_.end()).norm() < acceptance_radius_)
  {
    RCLCPP_INFO(get_logger(), "GOAL REACHED");
    finishGoal(true, FollowPath::Result::NONE, "");
    enterHover(traj_.end());
  } else if (stop_and_hover_ && stopped_at_end) {
    enterHover(p);
    finishGoal(false, FollowPath::Result::CANCELED, "canceled", true);
  }
}

void ControllerServer::finishGoal(bool success, uint16_t code, const std::string & msg, bool canceled)
{
  if (!goal_) {return;}
  const auto gh = std::move(goal_);
  goal_.reset();
  if (!gh->is_active()) {return;}
  auto r = std::make_shared<FollowPath::Result>();
  r->error_code = code;
  r->error_msg = msg;
  if (success) {
    gh->succeed(r);
  } else if (canceled && gh->is_canceling()) {
    gh->canceled(r);
  } else {
    gh->abort(r);
  }
}

void ControllerServer::publishFeedback()
{
  if (!goal_ || !goal_->is_executing()) {return;}
  auto fb = std::make_shared<FollowPath::Feedback>();
  const bool following = state_ == State::FOLLOW_PATH && !traj_.empty();
  fb->distance_remaining = following ? std::max(0.0, traj_.length() - s_ref_) : 0.0;
  fb->speed = v_ref_;
  fb->blocked = blocked_;
  fb->blocked_distance = blocked_ ? std::max(0.0, blocked_s_ - s_ref_) : 0.0;
  goal_->publish_feedback(fb);
}

void ControllerServer::publishState(bool force)
{
  const rclcpp::Time t = now();
  if (!force && state_ == last_published_state_ && last_state_publish_ &&
    (t - *last_state_publish_).seconds() < 0.5)
  {
    return;
  }
  last_state_publish_ = t;
  last_published_state_ = state_;
  State msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = t;
  msg.state = state_;
  msg.position.x = position_.x();
  msg.position.y = position_.y();
  msg.position.z = position_.z();
  msg.hold_position.x = hold_position_.x();
  msg.hold_position.y = hold_position_.y();
  msg.hold_position.z = hold_position_.z();
  state_pub_->publish(msg);
}

void ControllerServer::publishTrajectory()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  for (const auto & p : traj_.points()) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  traj_pub_->publish(path);
}

void ControllerServer::requestMapReset()
{
  if (!reset_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is not available: map not reset", reset_service_.c_str());
    return;
  }
  reset_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
}

// ============================================================================
// PX4 helpers
// ============================================================================

uint64_t ControllerServer::px4Timestamp() const
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
}

bool ControllerServer::isOffboard() const
{
  return nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool ControllerServer::isArmed() const
{
  return arming_state_ == VehicleStatus::ARMING_STATE_ARMED;
}

void ControllerServer::publishOffboardControlMode()
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

void ControllerServer::publishSetpoint(
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

void ControllerServer::publishVehicleCommand(uint32_t command, float p1, float p2)
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

void ControllerServer::engageOffboardAndArm()
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

}  // namespace uav_controller_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uav_controller_server::ControllerServer)
