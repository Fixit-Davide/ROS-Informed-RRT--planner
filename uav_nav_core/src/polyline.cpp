#include "uav_nav_core/polyline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uav_nav_core
{

double polylineLength(const std::vector<Eigen::Vector3d> & pts)
{
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {len += (pts[i] - pts[i - 1]).norm();}
  return len;
}

Eigen::Vector3d pointAlong(const std::vector<Eigen::Vector3d> & pts, double s)
{
  if (pts.empty()) {return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());}
  if (s <= 0.0) {return pts.front();}
  for (size_t i = 1; i < pts.size(); ++i) {
    const double seg = (pts[i] - pts[i - 1]).norm();
    if (s <= seg) {return seg > 0.0 ? pts[i - 1] + (pts[i] - pts[i - 1]) * (s / seg) : pts[i];}
    s -= seg;
  }
  return pts.back();
}

std::vector<Eigen::Vector3d> truncate(const std::vector<Eigen::Vector3d> & pts, double s)
{
  if (pts.empty()) {return {};}
  std::vector<Eigen::Vector3d> out{pts.front()};
  double acc = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double seg = (pts[i] - pts[i - 1]).norm();
    if (acc + seg >= s) {
      out.push_back(pointAlong(pts, s));
      return out;
    }
    acc += seg;
    out.push_back(pts[i]);
  }
  return out;
}

std::vector<Eigen::Vector3d> densify(const std::vector<Eigen::Vector3d> & pts, double spacing)
{
  if (spacing <= 0.0 || pts.size() < 2) {return pts;}
  std::vector<Eigen::Vector3d> out{pts.front()};
  for (size_t i = 1; i < pts.size(); ++i) {
    const double len = (pts[i] - pts[i - 1]).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / spacing)));
    for (int k = 1; k <= n; ++k) {out.push_back(pts[i - 1] + (pts[i] - pts[i - 1]) * (double(k) / n));}
  }
  return out;
}

}  // namespace uav_nav_core
