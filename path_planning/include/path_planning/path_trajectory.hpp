// Arc-length parameterised path with a speed limit profile, for smooth path following.
#ifndef PATH_PLANNING__PATH_TRAJECTORY_HPP_
#define PATH_PLANNING__PATH_TRAJECTORY_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace path_planning
{

struct TrajectoryLimits
{
  double max_speed{1.5};          // [m/s]
  double max_accel{1.0};          // [m/s^2] along the path
  double max_lateral_accel{1.0};  // [m/s^2] in corners
  double max_jerk{2.0};           // [m/s^3] rate of change of the along-path acceleration
};

// A polyline parameterised by arc length s in [0, length()].
//
// speedLimit(s) is the highest speed at s from which the vehicle can still slow down, at
// max_accel, to every corner speed ahead and to rest at the end. Corner speeds come from
// the lateral acceleration limit with the discrete curvature turn_angle / segment_length.
// Acceleration is not part of the profile: the follower limits it while integrating.
class PathTrajectory
{
public:
  PathTrajectory() = default;
  PathTrajectory(std::vector<Eigen::Vector3d> points, const TrajectoryLimits & limits);

  bool empty() const {return points_.size() < 2;}
  double length() const {return s_.empty() ? 0.0 : s_.back();}
  const std::vector<Eigen::Vector3d> & points() const {return points_;}
  const Eigen::Vector3d & end() const {return points_.back();}

  Eigen::Vector3d position(double s) const;
  Eigen::Vector3d tangent(double s) const;  // unit vector along the path at s
  double speedLimit(double s) const;
  // Index i of the segment [points[i], points[i+1]] that contains s.
  std::size_t segmentIndex(double s) const;
  // Arc length of the vertex i.
  double vertexArcLength(std::size_t i) const {return s_.at(i);}
  // Arc length of the point of the path closest to p.
  double project(const Eigen::Vector3d & p) const;
  // Points of the path between arc lengths s0 and s1 (both ends included).
  std::vector<Eigen::Vector3d> slice(double s0, double s1) const;

private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> s_;     // arc length of each vertex
  std::vector<double> vmax_;  // speed limit at each vertex (after the backward pass)
  TrajectoryLimits limits_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__PATH_TRAJECTORY_HPP_
