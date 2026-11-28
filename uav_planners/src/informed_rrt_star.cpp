#include "uav_planners/informed_rrt_star.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>
#include <rclcpp/rclcpp.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace uav_planners
{

namespace
{

Eigen::Vector3d toVec(const ob::State * s)
{
  const auto * rv = s->as<ob::RealVectorStateSpace::StateType>();
  return {rv->values[0], rv->values[1], rv->values[2]};
}

template<typename T>
T declareOrGet(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & def)
{
  if (!node.has_parameter(name)) {node.declare_parameter<T>(name, def);}
  return node.get_parameter(name).get_value<T>();
}

// Penalises states close to obstacles: cost = 1 / (clearance + eps), integrated along the path.
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
  ClearanceObjective(
    const ob::SpaceInformationPtr & si, std::shared_ptr<const uav_nav_core::CollisionWorld> w)
  : ob::StateCostIntegralObjective(si, true), world_(std::move(w)) {}

  ob::Cost stateCost(const ob::State * s) const override
  {
    return ob::Cost(1.0 / (std::min(world_->clearance(toVec(s)), 10.0) + 0.1));
  }

private:
  std::shared_ptr<const uav_nav_core::CollisionWorld> world_;
};

}  // namespace

void InformedRRTStar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name)
{
  auto node = parent.lock();
  if (!node) {throw std::runtime_error("InformedRRTStar: the parent node is gone");}
  logger_ = node->get_logger().get_child(name);
  range_ = declareOrGet<double>(*node, name + ".range", 4.0);
  k_nearest_ = declareOrGet<bool>(*node, name + ".k_nearest", false);
  objective_ = declareOrGet<std::string>(*node, name + ".objective", "path_length_cost_to_go");
  cost_threshold_ = declareOrGet<double>(*node, name + ".cost_threshold", 10.0);
  length_weight_ = declareOrGet<double>(*node, name + ".length_weight", 10.0);
  clearance_weight_ = declareOrGet<double>(*node, name + ".clearance_weight", 1.0);
  check_resolution_ = declareOrGet<double>(*node, name + ".collision_check_resolution", 0.1);
  goal_tolerance_ = declareOrGet<double>(*node, name + ".goal_tolerance", 0.3);
  smooth_ = declareOrGet<bool>(*node, name + ".smooth", true);
  if (objective_ != "path_length" && objective_ != "path_length_cost_to_go" &&
    objective_ != "threshold" && objective_ != "balanced")
  {
    throw std::invalid_argument("unknown " + name + ".objective '" + objective_ + "'");
  }
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  RCLCPP_INFO(
    logger_, "%s, objective %s, range %.1f m", objective_ == "balanced" ? "RRT*" : "Informed-RRT*",
    objective_.c_str(), range_);
}

std::vector<Eigen::Vector3d> InformedRRTStar::plan(
  const std::shared_ptr<const uav_nav_core::CollisionWorld> & w,
  const uav_nav_core::Bounds & window, const Eigen::Vector3d & start_pos,
  const Eigen::Vector3d & goal_pos, double solve_time, const std::function<bool()> & cancelled)
{
  const Eigen::Vector3d & lo = window.min, & hi = window.max;
  auto space = std::make_shared<ob::RealVectorStateSpace>(3);
  ob::RealVectorBounds bounds(3);
  for (int i = 0; i < 3; ++i) {
    bounds.setLow(i, lo[i]);
    bounds.setHigh(i, hi[i]);
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
    start[i] = std::clamp(start_pos[i], lo[i], hi[i]);
    goal[i] = std::clamp(goal_pos[i], lo[i], hi[i]);
  }

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start, goal, goal_tolerance_);
  // Start may lie inside an inflated voxel (e.g. close to a wall): nudge it out.
  pdef->fixInvalidInputStates(1.2, 1.2, 100);

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
  } else {  // balanced
    auto multi = std::make_shared<ob::MultiOptimizationObjective>(si);
    multi->addObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si), length_weight_);
    multi->addObjective(std::make_shared<ClearanceObjective>(si, w), clearance_weight_);
    obj = multi;
    informed = false;  // informed sampling is only defined for path length objectives
  }
  pdef->setOptimizationObjective(obj);

  ob::PlannerPtr planner;
  if (informed) {
    auto p = std::make_shared<og::InformedRRTstar>(si);
    p->setRange(range_);
    p->setKNearest(k_nearest_);
    // OMPL 1.6's default NearestNeighborsGNAT returns duplicate motions from list() after
    // the start state is moved by fixInvalidInputStates(); RRTstar::freeMemory then frees
    // the same motion twice and segfaults. SqrtApprox has no such bug.
    p->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();
    planner = p;
  } else {
    auto p = std::make_shared<og::RRTstar>(si);
    p->setRange(range_);
    p->setKNearest(k_nearest_);
    p->setNearestNeighbors<ompl::NearestNeighborsSqrtApprox>();  // see InformedRRTstar above
    planner = p;
  }
  planner->setProblemDefinition(pdef);
  planner->setup();

  auto ptc = ob::plannerOrTerminationCondition(
    ob::timedPlannerTerminationCondition(solve_time),
    ob::PlannerTerminationCondition([&cancelled]() {return cancelled && cancelled();}));

  const auto t0 = std::chrono::steady_clock::now();
  const ob::PlannerStatus status = planner->solve(ptc);
  if (*non_finite > 0) {
    RCLCPP_WARN(logger_, "OMPL produced %zu non-finite states (rejected)", non_finite->load());
  }
  if (status != ob::PlannerStatus::EXACT_SOLUTION) {
    RCLCPP_WARN(
      logger_, "Planner status: %s (start %s, goal %s, window [%.1f %.1f %.1f]-[%.1f %.1f %.1f])",
      status.asString().c_str(), w->isFree(start_pos) ? "free" : "in collision",
      w->isFree(goal_pos) ? "free" : "in collision", lo.x(), lo.y(), lo.z(), hi.x(), hi.y(), hi.z());
    return {};
  }

  auto path = *pdef->getSolutionPath()->as<og::PathGeometric>();
  if (smooth_) {
    og::PathSimplifier simplifier(si);
    og::PathGeometric smoothed = path;
    simplifier.reduceVertices(smoothed);
    simplifier.smoothBSpline(smoothed, 2);
    if (smoothed.check()) {path = smoothed;}
  }
  std::vector<Eigen::Vector3d> out;
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {out.push_back(toVec(path.getState(i)));}
  RCLCPP_INFO(
    logger_, "Found local path: %zu vertices, %.2f m, %.0f ms", out.size(), path.length(),
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  return out;
}

}  // namespace uav_planners

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uav_planners::InformedRRTStar, uav_nav_core::LocalPlanner)
