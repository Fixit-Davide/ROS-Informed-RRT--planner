#include "path_planning/path_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace path_planning
{

PathTrajectory::PathTrajectory(std::vector<Eigen::Vector3d> points, const TrajectoryLimits & limits)
: limits_(limits)
{
  if (limits.max_speed <= 0.0 || limits.max_accel <= 0.0 || limits.max_lateral_accel <= 0.0) {
    throw std::invalid_argument("trajectory limits must be positive");
  }
  // Drop repeated points: they would give zero-length segments without a tangent.
  for (const auto & p : points) {
    if (points_.empty() || (p - points_.back()).norm() > 1e-6) {points_.push_back(p);}
  }
  if (points_.size() < 2) {
    points_.clear();
    return;
  }
  const std::size_t n = points_.size();
  s_.assign(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    s_[i] = s_[i - 1] + (points_[i] - points_[i - 1]).norm();
  }

  // Corner speeds: v = sqrt(a_lat / kappa), kappa ~ turn angle / mean adjacent segment length.
  vmax_.assign(n, limits.max_speed);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const Eigen::Vector3d d0 = points_[i] - points_[i - 1];
    const Eigen::Vector3d d1 = points_[i + 1] - points_[i];
    const double c = std::clamp(d0.normalized().dot(d1.normalized()), -1.0, 1.0);
    const double angle = std::acos(c);
    if (angle < 1e-6) {continue;}
    const double kappa = angle / (0.5 * (d0.norm() + d1.norm()));
    vmax_[i] = std::min(limits.max_speed, std::sqrt(limits.max_lateral_accel / kappa));
  }
  vmax_.back() = 0.0;

  // Backward pass: every vertex must be able to brake to the next one at max_accel.
  for (std::size_t i = n - 1; i-- > 0; ) {
    const double ds = s_[i + 1] - s_[i];
    vmax_[i] = std::min(vmax_[i], std::sqrt(vmax_[i + 1] * vmax_[i + 1] + 2.0 * limits.max_accel * ds));
  }
}

std::size_t PathTrajectory::segmentIndex(double s) const
{
  if (empty()) {return 0;}
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const std::size_t i = static_cast<std::size_t>(std::distance(s_.begin(), it));
  return std::clamp<std::size_t>(i == 0 ? 0 : i - 1, 0, points_.size() - 2);
}

Eigen::Vector3d PathTrajectory::position(double s) const
{
  if (empty()) {return points_.empty() ? Eigen::Vector3d::Zero() : points_.front();}
  s = std::clamp(s, 0.0, length());
  const std::size_t i = segmentIndex(s);
  const double ds = s_[i + 1] - s_[i];
  const double t = ds > 0.0 ? (s - s_[i]) / ds : 0.0;
  return points_[i] + t * (points_[i + 1] - points_[i]);
}

Eigen::Vector3d PathTrajectory::tangent(double s) const
{
  if (empty()) {return Eigen::Vector3d::UnitX();}
  const std::size_t i = segmentIndex(std::clamp(s, 0.0, length()));
  return (points_[i + 1] - points_[i]).normalized();
}

double PathTrajectory::speedLimit(double s) const
{
  if (empty()) {return 0.0;}
  s = std::clamp(s, 0.0, length());
  const std::size_t i = segmentIndex(s);
  const double v_next = vmax_[i + 1];
  const double braking = std::sqrt(v_next * v_next + 2.0 * limits_.max_accel * (s_[i + 1] - s));
  return std::min({limits_.max_speed, braking});
}

double PathTrajectory::project(const Eigen::Vector3d & p) const
{
  if (empty()) {return 0.0;}
  double best_s = 0.0;
  double best_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    const Eigen::Vector3d d = points_[i + 1] - points_[i];
    const double len2 = d.squaredNorm();
    const double t = std::clamp((p - points_[i]).dot(d) / len2, 0.0, 1.0);
    const double dist = (points_[i] + t * d - p).squaredNorm();
    if (dist < best_d) {
      best_d = dist;
      best_s = s_[i] + t * std::sqrt(len2);
    }
  }
  return best_s;
}

std::vector<Eigen::Vector3d> PathTrajectory::slice(double s0, double s1) const
{
  std::vector<Eigen::Vector3d> out;
  if (empty()) {return out;}
  s0 = std::clamp(s0, 0.0, length());
  s1 = std::clamp(s1, s0, length());
  out.push_back(position(s0));
  for (std::size_t i = 0; i < points_.size(); ++i) {
    if (s_[i] > s0 && s_[i] < s1) {out.push_back(points_[i]);}
  }
  out.push_back(position(s1));
  return out;
}

}  // namespace path_planning
