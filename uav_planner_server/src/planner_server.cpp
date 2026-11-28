#include "uav_planner_server/planner_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "uav_nav_core/polyline.hpp"

namespace uav_planner_server
{

namespace
{

Eigen::Vector3d toVec(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {throw std::invalid_argument(name + " must have 3 elements");}
  return {v[0], v[1], v[2]};
}

double secondsSince(std::chrono::steady_clock::time_point t0)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("planner_server", options)
{
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("topics.map", "voxel_map");
  declare_parameter<std::vector<double>>("robot.size", {0.90, 0.90, 0.60});
  declare_parameter<double>("robot.planning_margin", 0.15);
  declare_parameter<std::vector<double>>("bounds.min", {-20.0, -20.0, 0.5});
  declare_parameter<std::vector<double>>("bounds.max", {20.0, 20.0, 4.0});
  declare_parameter<bool>("use_global", true);
  declare_parameter<std::string>("global_planner.plugin", "uav_planners::AStarPlanner");
  declare_parameter<std::string>("local_planner.plugin", "uav_planners::InformedRRTStar");
  declare_parameter<double>("local.horizon", 10.0);
  declare_parameter<double>("local.window_margin", 2.0);
  declare_parameter<bool>("local.direct_if_visible", true);
  declare_parameter<bool>("local.fallback_to_route", true);
  declare_parameter<double>("local.solve_time", 0.5);
  declare_parameter<double>("waypoint_spacing", 1.0);
  declare_parameter<double>("collision_check_resolution", 0.1);
}

PlannerServer::~PlannerServer()
{
  stopWorker();
}

PlannerServer::CallbackReturn PlannerServer::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    map_frame_ = get_parameter("map_frame").as_string();
    map_topic_ = get_parameter("topics.map").as_string();
    const Eigen::Vector3d size = toVec(get_parameter("robot.size").as_double_array(), "robot.size");
    // Paths are planned with a slightly larger box than the one the controller checks them
    // with, so a few new voxels next to a fresh path do not immediately trigger a replan.
    plan_box_ = size +
      Eigen::Vector3d::Constant(2.0 * get_parameter("robot.planning_margin").as_double());
    bounds_.min = toVec(get_parameter("bounds.min").as_double_array(), "bounds.min");
    bounds_.max = toVec(get_parameter("bounds.max").as_double_array(), "bounds.max");
    use_global_ = get_parameter("use_global").as_bool();
    horizon_ = get_parameter("local.horizon").as_double();
    window_margin_ = get_parameter("local.window_margin").as_double();
    direct_if_visible_ = get_parameter("local.direct_if_visible").as_bool();
    fallback_to_route_ = get_parameter("local.fallback_to_route").as_bool();
    solve_time_ = get_parameter("local.solve_time").as_double();
    waypoint_spacing_ = get_parameter("waypoint_spacing").as_double();
    check_resolution_ = get_parameter("collision_check_resolution").as_double();

    global_loader_ = std::make_unique<pluginlib::ClassLoader<uav_nav_core::GlobalPlanner>>(
      "uav_nav_core", "uav_nav_core::GlobalPlanner");
    local_loader_ = std::make_unique<pluginlib::ClassLoader<uav_nav_core::LocalPlanner>>(
      "uav_nav_core", "uav_nav_core::LocalPlanner");
    if (use_global_) {
      global_ = global_loader_->createSharedInstance(get_parameter("global_planner.plugin").as_string());
      global_->configure(shared_from_this(), "global_planner");
    }
    local_ = local_loader_->createSharedInstance(get_parameter("local_planner.plugin").as_string());
    local_->configure(shared_from_this(), "local_planner");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    global_.reset();
    local_.reset();
    return CallbackReturn::FAILURE;
  }

  world_ = std::make_shared<const uav_nav_core::CollisionWorld>(nullptr, plan_box_);
  route_pub_ = create_publisher<nav_msgs::msg::Path>("~/global_path", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/local_path", rclcpp::QoS(1).transient_local());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/path_markers", rclcpp::QoS(1).transient_local());
  RCLCPP_INFO(
    get_logger(), "Configured: %s, horizon %.1f m, planning box %.2f x %.2f x %.2f m",
    use_global_ ? "global route + local path" : "local planner to the goal", horizon_,
    plan_box_.x(), plan_box_.y(), plan_box_.z());
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_activate(const rclcpp_lifecycle::State &)
{
  route_pub_->on_activate();
  path_pub_->on_activate();
  marker_pub_->on_activate();
  map_sub_ = create_subscription<uav_nav_core::VoxelMap>(
    map_topic_, uav_nav_core::voxelMapQoS(),
    [this](std::shared_ptr<const uav_nav_core::VoxelMap> m) {mapCallback(std::move(m));});

  stop_ = false;
  worker_ = std::thread(&PlannerServer::worker, this);
  action_server_ = rclcpp_action::create_server<ComputePath>(
    this, "compute_path",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePath::Goal>) {
      return stop_ ? rclcpp_action::GoalResponse::REJECT :
      rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<GoalHandle> gh) {
      // Latest goal wins: a goal still waiting is dropped, a running solve stops early.
      std::shared_ptr<GoalHandle> dropped;
      {
        std::lock_guard<std::mutex> lk(goal_mutex_);
        dropped = std::move(pending_);
        pending_ = gh;
      }
      if (dropped) {
        auto r = std::make_shared<ComputePath::Result>();
        r->error_code = ComputePath::Result::CANCELED;
        r->error_msg = "superseded by a newer goal";
        dropped->abort(r);
      }
      goal_cv_.notify_one();
    });
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  stopWorker();
  map_sub_.reset();
  route_pub_->on_deactivate();
  path_pub_->on_deactivate();
  marker_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (global_) {global_->cleanup();}
  if (local_) {local_->cleanup();}
  global_.reset();
  local_.reset();
  global_loader_.reset();
  local_loader_.reset();
  route_pub_.reset();
  path_pub_.reset();
  marker_pub_.reset();
  std::lock_guard<std::mutex> lk(world_mutex_);
  world_.reset();
  last_snapshot_.reset();
  return CallbackReturn::SUCCESS;
}

PlannerServer::CallbackReturn PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  action_server_.reset();
  stopWorker();
  map_sub_.reset();
  return CallbackReturn::SUCCESS;
}

void PlannerServer::stopWorker()
{
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    stop_ = true;
  }
  goal_cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
  std::shared_ptr<GoalHandle> dropped;
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    dropped = std::move(pending_);
  }
  if (dropped && dropped->is_active()) {
    auto r = std::make_shared<ComputePath::Result>();
    r->error_code = ComputePath::Result::CANCELED;
    r->error_msg = "planner server stopping";
    dropped->abort(r);
  }
}

void PlannerServer::mapCallback(std::shared_ptr<const uav_nav_core::VoxelMap> map)
{
  if (!map || !map->snapshot) {return;}
  if (!map->header.frame_id.empty() && map->header.frame_id != map_frame_ && !warned_frame_) {
    RCLCPP_WARN(
      get_logger(), "Map is in '%s', planning in '%s': set the same map_frame on both servers",
      map->header.frame_id.c_str(), map_frame_.c_str());
    warned_frame_ = true;
  }
  auto w = std::make_shared<const uav_nav_core::CollisionWorld>(map->snapshot, plan_box_);
  std::lock_guard<std::mutex> lk(world_mutex_);
  world_ = std::move(w);
  last_snapshot_ = map->snapshot;
}

std::shared_ptr<const uav_nav_core::CollisionWorld> PlannerServer::world() const
{
  std::lock_guard<std::mutex> lk(world_mutex_);
  return world_;
}

bool PlannerServer::superseded() const
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  return stop_ || pending_ != nullptr;
}

void PlannerServer::worker()
{
  while (true) {
    std::shared_ptr<GoalHandle> gh;
    {
      std::unique_lock<std::mutex> lk(goal_mutex_);
      goal_cv_.wait(lk, [this]() {return stop_ || pending_;});
      if (stop_) {return;}
      gh = std::move(pending_);
    }
    const auto goal = gh->get_goal();
    auto result = std::make_shared<ComputePath::Result>();
    const auto bad_frame = [this](const std::string & f) {return !f.empty() && f != map_frame_;};
    const Eigen::Vector3d start(goal->start.point.x, goal->start.point.y, goal->start.point.z);
    const Eigen::Vector3d target(goal->goal.point.x, goal->goal.point.y, goal->goal.point.z);
    if (!start.allFinite() || !target.allFinite() || bad_frame(goal->start.header.frame_id) ||
      bad_frame(goal->goal.header.frame_id))
    {
      result->error_code = ComputePath::Result::INVALID_REQUEST;
      result->error_msg = "start and goal must be finite and in " + map_frame_;
      gh->abort(result);
      continue;
    }
    const auto cancelled = [this, gh]() {return gh->is_canceling() || superseded();};
    Plan p;
    try {
      p = plan(start, target, goal->solve_time > 0.0 ? goal->solve_time : solve_time_, cancelled);
    } catch (const std::exception & e) {
      p.error = ComputePath::Result::NO_LOCAL_PATH;
      p.error_msg = std::string("planner exception: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", p.error_msg.c_str());
    }
    if (!p.route.empty()) {route_pub_->publish(toPath(p.route));}
    result->route = toPath(p.route);
    result->path = toPath(p.path);
    result->final = p.final;
    result->route_time = p.route_time;
    result->local_time = p.local_time;
    if (gh->is_canceling()) {
      result->error_code = ComputePath::Result::CANCELED;
      result->error_msg = "canceled";
      gh->canceled(result);
    } else if (p.error == ComputePath::Result::NONE && superseded()) {
      // A newer goal is waiting: this path is stale, do not hand it out.
      result->error_code = ComputePath::Result::CANCELED;
      result->error_msg = "superseded by a newer goal";
      gh->abort(result);
    } else if (p.error != ComputePath::Result::NONE) {
      if (superseded()) {
        p.error = ComputePath::Result::CANCELED;
        p.error_msg = "superseded by a newer goal";
      }
      result->error_code = p.error;
      result->error_msg = p.error_msg;
      gh->abort(result);
    } else {
      path_pub_->publish(result->path);
      publishMarkers(p.path);
      gh->succeed(result);
    }
  }
}

PlannerServer::Plan PlannerServer::plan(
  const Eigen::Vector3d & start_in, const Eigen::Vector3d & goal_in, double solve_time,
  const std::function<bool()> & cancelled)
{
  Plan out;
  const auto w = world();
  if (!w->hasMap()) {
    RCLCPP_WARN(get_logger(), "Map is empty (no lidar scan yet): planning in free space");
  }
  const Eigen::Vector3d start = start_in.cwiseMax(bounds_.min).cwiseMin(bounds_.max);
  const Eigen::Vector3d goal = goal_in.cwiseMax(bounds_.min).cwiseMin(bounds_.max);
  const auto t_local = [&](std::chrono::steady_clock::time_point t0) {out.local_time = secondsSince(t0);};

  if (!use_global_) {
    const auto t0 = std::chrono::steady_clock::now();
    out.path = uav_nav_core::densify(
      local_->plan(w, bounds_, start, goal, solve_time, cancelled), waypoint_spacing_);
    t_local(t0);
    if (out.path.empty()) {
      out.error = cancelled() ? ComputePath::Result::CANCELED : ComputePath::Result::NO_LOCAL_PATH;
      out.error_msg = "local planner found no path to the goal";
    }
    return out;
  }

  // ---- global: route over the whole planning volume ----
  const auto t0 = std::chrono::steady_clock::now();
  out.route = global_->plan(*w, bounds_, start, goal);
  out.route_time = secondsSince(t0);
  if (out.route.empty()) {
    out.error = ComputePath::Result::NO_ROUTE;
    out.error_msg = "no global route to the goal";
    return out;
  }

  // ---- local: path to the route point local.horizon ahead, in a window around that part
  // of the route ----
  const double route_len = uav_nav_core::polylineLength(out.route);
  out.final = route_len <= horizon_ + 1e-6;
  const auto part = uav_nav_core::truncate(out.route, out.final ? route_len : horizon_);
  const Eigen::Vector3d local_goal = part.back();
  Eigen::Vector3d lo = local_goal, hi = local_goal;
  for (const auto & p : part) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  uav_nav_core::Bounds window;
  window.min = (lo - Eigen::Vector3d::Constant(window_margin_)).cwiseMax(bounds_.min);
  window.max = (hi + Eigen::Vector3d::Constant(window_margin_)).cwiseMin(bounds_.max);
  window.min.z() = bounds_.min.z();  // the full altitude band: the local plan may climb over things
  window.max.z() = bounds_.max.z();
  RCLCPP_INFO(
    get_logger(), "Route %.1f m; local goal %.1f m ahead%s", route_len,
    out.final ? route_len : horizon_, out.final ? " (the goal)" : "");

  const auto t1 = std::chrono::steady_clock::now();
  std::vector<Eigen::Vector3d> path;
  if (direct_if_visible_ && w->isFree(start) &&
    w->isSegmentFree(start, local_goal, check_resolution_))
  {
    // Nothing between here and the local goal: the straight line is what Informed-RRT* would
    // converge to, so skip the solve.
    path = {start, local_goal};
    RCLCPP_INFO(
      get_logger(), "Local path: straight line to the local goal (%.2f m)", (local_goal - start).norm());
  } else {
    path = local_->plan(w, window, start, local_goal, solve_time, cancelled);
  }
  if (path.empty() && fallback_to_route_ && !cancelled()) {
    // The route itself is collision free for the planning box (A* moves and shortcuts are
    // checked), so fly it up to the local goal rather than giving up.
    path = part;
    RCLCPP_WARN(
      get_logger(), "Local planner found no path: following the global route (%.1f m)",
      uav_nav_core::polylineLength(part));
  }
  t_local(t1);
  if (path.empty()) {
    out.error = cancelled() ? ComputePath::Result::CANCELED : ComputePath::Result::NO_LOCAL_PATH;
    out.error_msg = "local planner found no path along the route";
    return out;
  }
  out.path = uav_nav_core::densify(path, waypoint_spacing_);
  return out;
}

nav_msgs::msg::Path PlannerServer::toPath(const std::vector<Eigen::Vector3d> & pts) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = now();
  path.poses.reserve(pts.size());
  for (const auto & p : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

void PlannerServer::publishMarkers(const std::vector<Eigen::Vector3d> & pts)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker spheres;
  spheres.header.frame_id = map_frame_;
  spheres.header.stamp = now();
  spheres.ns = "waypoints";
  spheres.id = 0;
  spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::msg::Marker::ADD;
  spheres.pose.orientation.w = 1.0;
  spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.25;
  spheres.color.g = 0.8f;
  spheres.color.a = 0.8f;
  for (const auto & p : pts) {
    geometry_msgs::msg::Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    spheres.points.push_back(q);
  }
  auto line = spheres;
  line.ns = "path";
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.scale.x = 0.05;
  line.color.r = 0.2f;
  markers.markers.push_back(spheres);
  markers.markers.push_back(line);
  marker_pub_->publish(markers);
}

}  // namespace uav_planner_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uav_planner_server::PlannerServer)
