#include "uav_nav_core/collision_world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace uav_nav_core
{

CollisionWorld::CollisionWorld(
  std::shared_ptr<const OccupancySnapshot> map, const Eigen::Vector3d & box_size)
: map_(std::move(map)), half_(0.5 * box_size)
{
}

bool CollisionWorld::isFree(const Eigen::Vector3d & p) const
{
  // A NaN position would turn into an INT_MIN..INT_MAX voxel loop; FCL just said "free".
  if (!p.allFinite()) {return false;}
  if (!hasMap()) {return true;}
  // Voxel k spans [k * res, (k + 1) * res). The small epsilon keeps a box face lying exactly
  // on a voxel boundary from counting the voxel beyond it (FCL treats touching as free too).
  constexpr double kEps = 1e-9;
  const double inv = 1.0 / map_->resolution();
  const Eigen::Vector3d lo = (p - half_) * inv;
  const Eigen::Vector3d hi = (p + half_) * inv;
  // Voxel indices are int32: refuse positions that would overflow them.
  constexpr double kMaxIndex = 1e9;
  if (lo.cwiseAbs().maxCoeff() > kMaxIndex || hi.cwiseAbs().maxCoeff() > kMaxIndex) {return false;}
  const Bonxai::CoordT c_lo{
    static_cast<int32_t>(std::floor(lo.x() + kEps)),
    static_cast<int32_t>(std::floor(lo.y() + kEps)),
    static_cast<int32_t>(std::floor(lo.z() + kEps))};
  const Bonxai::CoordT c_hi{
    static_cast<int32_t>(std::ceil(hi.x() - kEps)) - 1,
    static_cast<int32_t>(std::ceil(hi.y() - kEps)) - 1,
    static_cast<int32_t>(std::ceil(hi.z() - kEps)) - 1};
  return !map_->anyOccupied(c_lo, c_hi);
}

double CollisionWorld::clearance(const Eigen::Vector3d & p) const
{
  if (!p.allFinite()) {return 0.0;}
  if (!hasMap()) {return std::numeric_limits<double>::max();}
  // Box-to-voxel distance, with the voxel as a cube of edge `res` around its centre.
  const Eigen::Vector3d reach = half_ + Eigen::Vector3d::Constant(0.5 * map_->resolution());
  const Eigen::Vector3d box_min = p - half_;
  const Eigen::Vector3d box_max = p + half_;
  double best2 = std::numeric_limits<double>::max();
  for (const auto & b : map_->blocks()) {
    // Lower bound: distance between the box and the block's bounds.
    const Eigen::Vector3d gap =
      (b.min - box_max).cwiseMax(box_min - b.max).cwiseMax(Eigen::Vector3d::Zero());
    if (gap.squaredNorm() >= best2) {continue;}
    for (const auto & c : b.centres) {
      const Eigen::Vector3d d = ((c - p).cwiseAbs() - reach).cwiseMax(Eigen::Vector3d::Zero());
      best2 = std::min(best2, d.squaredNorm());
    }
    if (best2 == 0.0) {return 0.0;}
  }
  return std::sqrt(best2);
}

bool CollisionWorld::isSegmentFree(
  const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const
{
  if (!hasMap()) {return true;}
  const double len = (b - a).norm();
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(step, 1e-3))));
  for (int i = 0; i <= n; ++i) {
    if (!isFree(a + (b - a) * (static_cast<double>(i) / n))) {return false;}
  }
  return true;
}

}  // namespace uav_nav_core
