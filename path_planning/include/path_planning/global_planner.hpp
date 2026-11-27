// Global route planner: A* over a coarse 3D lattice of robot positions, checked against a
// CollisionWorld, followed by line-of-sight shortcutting. The route is only a guide for the
// local Informed-RRT* planner, which plans the flyable path along it.
#ifndef PATH_PLANNING__GLOBAL_PLANNER_HPP_
#define PATH_PLANNING__GLOBAL_PLANNER_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "path_planning/collision_world.hpp"

namespace path_planning
{

struct GlobalPlannerOptions
{
  double resolution{0.4};         // [m] lattice spacing; keep it <= the smallest box size
  double snap_radius{1.2};        // [m] how far an invalid start / goal may be moved
  double shortcut_step{0.1};      // [m] sampling step of the line-of-sight checks
  size_t max_expansions{500000};  // give up after expanding this many nodes
};

struct GlobalPlan
{
  bool success{false};
  std::vector<Eigen::Vector3d> path;  // start, shortcut lattice nodes, goal
  double length{0.0};                 // [m]
  size_t expansions{0};
};

class GlobalPlanner
{
public:
  explicit GlobalPlanner(const GlobalPlannerOptions & options) : options_(options) {}

  // Route from start to goal inside [bounds_min, bounds_max]. Lattice nodes are positions of
  // the robot box; a node is usable when world.isFree() holds there. Moves go to the 26
  // neighbours without cutting corners, cost = Euclidean length, heuristic = Euclidean
  // distance to the goal. Shortcuts are checked with isSegmentFree() every shortcut_step.
  GlobalPlan plan(
    const CollisionWorld & world, const Eigen::Vector3d & bounds_min,
    const Eigen::Vector3d & bounds_max, const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal) const;

private:
  GlobalPlannerOptions options_;
};

// Total length of a polyline.
double polylineLength(const std::vector<Eigen::Vector3d> & pts);
// Point at arc length s along a polyline (clamped to its ends).
Eigen::Vector3d pointAlong(const std::vector<Eigen::Vector3d> & pts, double s);

}  // namespace path_planning

#endif  // PATH_PLANNING__GLOBAL_PLANNER_HPP_
