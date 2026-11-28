// Small helpers on polylines (routes and paths as vertex lists).
#ifndef UAV_NAV_CORE__POLYLINE_HPP_
#define UAV_NAV_CORE__POLYLINE_HPP_

#include <vector>

#include <Eigen/Core>

namespace uav_nav_core
{

// Total length of a polyline.
double polylineLength(const std::vector<Eigen::Vector3d> & pts);
// Point at arc length s along a polyline (clamped to its ends).
Eigen::Vector3d pointAlong(const std::vector<Eigen::Vector3d> & pts, double s);
// The polyline up to arc length s (its first vertices, then the point at s).
std::vector<Eigen::Vector3d> truncate(const std::vector<Eigen::Vector3d> & pts, double s);
// Same polyline with each segment split into ceil(length / spacing) equal parts, so no two
// consecutive points are more than `spacing` apart (spacing <= 0: unchanged).
std::vector<Eigen::Vector3d> densify(const std::vector<Eigen::Vector3d> & pts, double spacing);

}  // namespace uav_nav_core

#endif  // UAV_NAV_CORE__POLYLINE_HPP_
