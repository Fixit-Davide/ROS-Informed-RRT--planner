#include "uav_navigator/navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.hpp>

using namespace std::chrono_literals;

namespace uav_navigator
{

namespace
{

Eigen::Vector3d toVec(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {throw std::invalid_argument(name + " must have 3 elements");}
  return {v[0], v[1], v[2]};
}

constexpr uint16_t kNavUnknown = nav2_msgs::action::NavigateToPose::Result::UNKNOWN;
constexpr uint16_t kNavTfError = nav2_msgs::action::NavigateToPose::Result::TF_ERROR;

}  // namespace

Navigator::Navigator(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("navigator", options)
{
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::vector<double>>("bounds.min", {-20.0, -20.0, 0.5});
  declare_parameter<std::vector<double>>("bounds.max", {20.0, 20.0, 4.0});
  declare_parameter<double>("default_goal_altitude", 2.0);
  declare_parameter<double>("solve_time", 0.5);
  declare_parameter<int>("max_attempts", 3);
  declare_parameter<double>("replan_distance", 5.0);
  declare_parameter<std::string>("topics.goal", "/goal_pose");
  declare_parameter<std::string>("topics.controller_state", "/controller_server/state");
  declare_parameter<std::string>("compute_path_action", "compute_path");
  declare_parameter<std::string>("follow_path_action", "follow_path");
  declare_parameter<std::string>("replan_start_service", "/controller_server/get_replan_start");
}

// ============================================================================
// Lifecycle
// ============================================================================

Navigator::CallbackReturn Navigator::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    bounds_min_ = toVec(get_parameter("bounds.min").as_double_array(), "bounds.min");
    bounds_max_ = toVec(get_parameter("bounds.max").as_double_array(), "bounds.max");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  map_frame_ = get_parameter("map_frame").as_string();
  default_goal_altitude_ = get_parameter("default_goal_altitude").as_double();
  solve_time_ = get_parameter("solve_time").as_double();
  max_attempts_ = std::max(1, static_cast<int>(get_parameter("max_attempts").as_int()));
  replan_distance_ = get_parameter("replan_distance").as_double();
  goal_topic_ = get_parameter("topics.goal").as_string();
  controller_state_topic_ = get_parameter("topics.controller_state").as_string();
  compute_path_action_ = get_parameter("compute_path_action").as_string();
  follow_path_action_ = get_parameter("follow_path_action").as_string();
  replan_service_ = get_parameter("replan_start_service").as_string();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  plan_client_ = rclcpp_action::create_client<ComputePath>(this, compute_path_action_);
  follow_client_ = rclcpp_action::create_client<FollowPath>(this, follow_path_action_);
  replan_client_ = create_client<GetReplanStart>(replan_service_);
  RCLCPP_INFO(
    get_logger(), "Configured: next segment planned %.1f m before the end, %d attempts from %.1f s",
    replan_distance_, max_attempts_, solve_time_);
  return CallbackReturn::SUCCESS;
}

Navigator::CallbackReturn Navigator::on_activate(const rclcpp_lifecycle::State &)
{
  state_sub_ = create_subscription<ControllerState>(
    controller_state_topic_, rclcpp::QoS(1).transient_local(),
    [this](ControllerState::SharedPtr m) {stateCallback(m);});
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(1),
    [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {goalTopicCallback(m);});
  nav_server_ = rclcpp_action::create_server<NavigateToPose>(
    this, "navigate_to_pose",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<NavGoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<NavGoalHandle> gh) {acceptNavigation(gh);});
  timer_ = create_wall_timer(100ms, [this]() {tick();});
  return CallbackReturn::SUCCESS;
}

Navigator::CallbackReturn Navigator::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (goal_) {
    stopVehicle();
    finishNavigation(false, kNavUnknown, "navigator deactivated");
  }
  timer_.reset();
  nav_server_.reset();
  goal_sub_.reset();
  state_sub_.reset();
  controller_state_.reset();
  return CallbackReturn::SUCCESS;
}

Navigator::CallbackReturn Navigator::on_cleanup(const rclcpp_lifecycle::State &)
{
  plan_client_.reset();
  follow_client_.reset();
  replan_client_.reset();
  follow_goal_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  return CallbackReturn::SUCCESS;
}

Navigator::CallbackReturn Navigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  nav_server_.reset();
  return CallbackReturn::SUCCESS;
}

// ============================================================================
// Goals
// ============================================================================

std::optional<Eigen::Vector3d> Navigator::toMapGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped goal = pose;
  if (!goal.header.frame_id.empty() && goal.header.frame_id != map_frame_) {
    try {
      goal = tf_buffer_->transform(goal, map_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "Cannot transform goal into '%s': %s", map_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }
  Eigen::Vector3d g(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  if (!g.allFinite()) {return std::nullopt;}
  // RViz "2D Goal Pose" has z = 0: replace it with the configured altitude.
  if (g.z() < bounds_min_.z()) {g.z() = default_goal_altitude_;}
  return g.cwiseMax(bounds_min_).cwiseMin(bounds_max_);
}

void Navigator::goalTopicCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const auto g = toMapGoal(*msg);
  if (!g) {return;}
  if (nav_goal_) {finishNavigation(false, kNavUnknown, "replaced by a goal on " + goal_topic_);}
  startNavigation(*g);
}

void Navigator::acceptNavigation(const std::shared_ptr<NavGoalHandle> & gh)
{
  const auto g = toMapGoal(gh->get_goal()->pose);
  if (!g) {
    auto r = std::make_shared<NavigateToPose::Result>();
    r->error_code = kNavTfError;
    r->error_msg = "goal cannot be expressed in " + map_frame_;
    gh->abort(r);
    return;
  }
  if (nav_goal_) {finishNavigation(false, kNavUnknown, "replaced by a newer goal");}
  nav_goal_ = gh;
  startNavigation(*g);
}

void Navigator::startNavigation(const Eigen::Vector3d & g)
{
  goal_ = g;
  replans_ = 0;
  nav_start_ = now();
  ++nav_id_;
  RCLCPP_INFO(get_logger(), "New goal: [%.2f, %.2f, %.2f]", g.x(), g.y(), g.z());
  const uint8_t state = controller_state_ ? controller_state_->state : ControllerState::WAIT_FOR_ODOM;
  if (state == ControllerState::FOLLOW_PATH) {
    // Keep flying the current path while the new one is planned, then splice onto it.
    requestPlan(PlanKind::NEW_GOAL);
  } else if (state == ControllerState::HOVER) {
    requestPlan(PlanKind::FROM_HOVER);
  } else {
    ++plan_seq_;  // drop anything still in flight for an older goal
    plan_in_flight_ = false;
    waiting_for_hover_ = true;
    RCLCPP_INFO(get_logger(), "Vehicle not airborne yet: planning will start after takeoff");
  }
}

void Navigator::finishNavigation(bool success, uint16_t code, const std::string & msg)
{
  goal_.reset();
  waiting_for_hover_ = false;
  ++plan_seq_;
  plan_in_flight_ = false;
  if (!nav_goal_) {return;}
  const auto gh = std::move(nav_goal_);
  nav_goal_.reset();
  if (!gh->is_active()) {return;}
  auto r = std::make_shared<NavigateToPose::Result>();
  r->error_code = success ? NavigateToPose::Result::NONE : code;
  r->error_msg = msg;
  if (success) {
    gh->succeed(r);
  } else if (gh->is_canceling()) {
    gh->canceled(r);
  } else {
    gh->abort(r);
  }
}

// ============================================================================
// Planning and following
// ============================================================================

double Navigator::solveTime(int attempt) const
{
  return solve_time_ * std::pow(2.0, attempt - 1);
}

void Navigator::stateCallback(const ControllerState::SharedPtr msg)
{
  controller_state_ = *msg;
  if (waiting_for_hover_ && goal_ && msg->state == ControllerState::HOVER) {
    waiting_for_hover_ = false;
    requestPlan(PlanKind::FROM_HOVER);
  }
}

void Navigator::requestPlan(PlanKind kind, int attempt)
{
  if (!goal_) {return;}
  if (!replan_client_->service_is_ready() || !plan_client_->action_server_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "%s or %s is not available: navigation aborted", replan_service_.c_str(),
      compute_path_action_.c_str());
    stopVehicle();
    finishNavigation(false, kNavUnknown, "planner or controller server not available");
    return;
  }
  const uint64_t seq = ++plan_seq_;
  plan_in_flight_ = true;
  plan_kind_ = kind;
  attempt_ = attempt;

  // Ask the controller where the plan must start: where the reference will be when the plan
  // is ready (it keeps moving meanwhile), the end of the current segment, or the hold point.
  auto req = std::make_shared<GetReplanStart::Request>();
  req->lookahead_time = solveTime(attempt) + 0.2;
  req->from_end = kind == PlanKind::NEXT_SEGMENT;
  replan_client_->async_send_request(
    req, [this, seq](rclcpp::Client<GetReplanStart>::SharedFuture f) {
      if (seq != plan_seq_ || !goal_) {return;}
      const auto res = f.get();
      ComputePath::Goal g;
      g.start.header.frame_id = map_frame_;
      g.start.point = res->start;
      g.goal.header.frame_id = map_frame_;
      g.goal.point.x = goal_->x();
      g.goal.point.y = goal_->y();
      g.goal.point.z = goal_->z();
      g.solve_time = solveTime(attempt_);
      rclcpp_action::Client<ComputePath>::SendGoalOptions opts;
      opts.goal_response_callback = [this, seq](
        rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr gh) {
          if (!gh && seq == plan_seq_) {
            plan_in_flight_ = false;
            onPlanFailed("compute_path goal rejected");
          }
        };
      opts.result_callback = [this, seq](
        const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r) {
          onPlanResult(seq, r);
        };
      plan_client_->async_send_goal(g, opts);
    });
}

void Navigator::onPlanResult(
  uint64_t seq, const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r)
{
  if (seq != plan_seq_ || !goal_) {return;}  // superseded by a newer request
  plan_in_flight_ = false;
  if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result) {
    sendFollow(r.result->path, r.result->final);
    return;
  }
  onPlanFailed(r.result ? r.result->error_msg : "compute_path failed");
}

void Navigator::onPlanFailed(const std::string & why)
{
  const bool moving = follow_goal_ && controller_state_ &&
    controller_state_->state == ControllerState::FOLLOW_PATH;
  if (attempt_ < max_attempts_) {
    RCLCPP_WARN(
      get_logger(), "No path (%s), attempt %d/%d: retrying with %.1f s", why.c_str(), attempt_,
      max_attempts_, solveTime(attempt_ + 1));
    // While moving, retry from just ahead of the reference (the end of the segment may be
    // the problem); from hover, from the hold point.
    const PlanKind kind = !moving ? PlanKind::FROM_HOVER :
      plan_kind_ == PlanKind::BLOCKED ? PlanKind::BLOCKED : PlanKind::NEW_GOAL;
    requestPlan(kind, attempt_ + 1);
    return;
  }
  if (moving) {
    RCLCPP_WARN(get_logger(), "No path found: stopping along the path, then hovering");
    stopVehicle();
  } else {
    RCLCPP_WARN(get_logger(), "No path found, hovering");
  }
  finishNavigation(false, kNavUnknown, "no path to the goal: " + why);
}

void Navigator::sendFollow(const nav_msgs::msg::Path & path, bool final)
{
  if (!follow_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "%s is not available: navigation aborted", follow_path_action_.c_str());
    finishNavigation(false, kNavUnknown, "controller server not available");
    return;
  }
  FollowPath::Goal g;
  g.path = path;
  g.final = final;
  segment_final_ = final;
  follow_pending_ = true;
  follow_nav_id_ = nav_id_;
  rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
  opts.goal_response_callback = [this](FollowGoalHandle::SharedPtr gh) {
      follow_pending_ = false;
      if (!gh) {
        RCLCPP_ERROR(get_logger(), "follow_path goal rejected");
        finishNavigation(false, kNavUnknown, "follow_path goal rejected");
        return;
      }
      follow_goal_ = gh;
    };
  opts.feedback_callback = [this](
    FollowGoalHandle::SharedPtr gh, const std::shared_ptr<const FollowPath::Feedback> fb) {
      onFollowFeedback(gh, fb);
    };
  opts.result_callback = [this](const FollowGoalHandle::WrappedResult & r) {onFollowResult(r);};
  follow_client_->async_send_goal(g, opts);
}

void Navigator::onFollowFeedback(
  FollowGoalHandle::SharedPtr gh, const std::shared_ptr<const FollowPath::Feedback> fb)
{
  if (follow_pending_ || gh != follow_goal_ || !goal_) {return;}
  distance_remaining_ = fb->distance_remaining;
  if (fb->blocked) {
    // A plan of the next segment starts beyond the block: replace it. Other plans in flight
    // start before the braking point and are planned against the current map: keep them.
    if (!plan_in_flight_ || plan_kind_ == PlanKind::NEXT_SEGMENT) {
      RCLCPP_WARN(get_logger(), "Path blocked %.1f m ahead: REPLANNING", fb->blocked_distance);
      ++replans_;
      requestPlan(PlanKind::BLOCKED);
    }
    return;
  }
  // Receding horizon: plan the next local segment while this one still has room ahead. It
  // starts at this segment's end, so the current segment is flown entirely.
  if (!segment_final_ && !plan_in_flight_ && fb->distance_remaining < replan_distance_) {
    requestPlan(PlanKind::NEXT_SEGMENT);
  }
}

void Navigator::onFollowResult(const FollowGoalHandle::WrappedResult & r)
{
  if (!follow_goal_ || r.goal_id != follow_goal_->get_goal_id()) {return;}
  follow_goal_.reset();
  const uint16_t code = r.result ? r.result->error_code : FollowPath::Result::NONE;
  switch (r.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // A path of an earlier goal may finish while the new goal is being planned.
      if (goal_ && follow_nav_id_ == nav_id_) {
        RCLCPP_INFO(get_logger(), "GOAL REACHED");
        finishNavigation(true, 0, "");
      }
      break;
    case rclcpp_action::ResultCode::CANCELED:
      break;  // we stopped it
    default:
      if (code == FollowPath::Result::REPLACED) {break;}
      if (code == FollowPath::Result::INVALID_PATH && goal_) {
        // Degenerate intermediate segment: plan again from where the vehicle holds.
        requestPlan(PlanKind::FROM_HOVER);
        break;
      }
      if (goal_) {
        RCLCPP_WARN(
          get_logger(), "Controller stopped the path: %s",
          r.result ? r.result->error_msg.c_str() : "unknown");
        finishNavigation(
          false, kNavUnknown, "controller: " + (r.result ? r.result->error_msg : std::string()));
      }
      break;
  }
}

void Navigator::stopVehicle()
{
  if (follow_goal_ && follow_client_) {follow_client_->async_cancel_goal(follow_goal_);}
}

void Navigator::tick()
{
  if (nav_goal_ && nav_goal_->is_canceling()) {
    RCLCPP_INFO(get_logger(), "Navigation canceled: stopping");
    stopVehicle();
    finishNavigation(false, kNavUnknown, "canceled");
    return;
  }
  if (nav_goal_ && nav_goal_->is_executing() && goal_) {
    auto fb = std::make_shared<NavigateToPose::Feedback>();
    fb->current_pose.header.frame_id = map_frame_;
    fb->current_pose.header.stamp = now();
    fb->current_pose.pose.orientation.w = 1.0;
    if (controller_state_) {
      fb->current_pose.pose.position = controller_state_->position;
      const auto & p = controller_state_->position;
      fb->distance_remaining = static_cast<float>(
        (Eigen::Vector3d(p.x, p.y, p.z) - *goal_).norm());
    }
    fb->navigation_time = rclcpp::Duration(now() - nav_start_);
    fb->number_of_recoveries = replans_;
    nav_goal_->publish_feedback(fb);
  }
}

}  // namespace uav_navigator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uav_navigator::Navigator)
