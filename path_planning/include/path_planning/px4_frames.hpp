// Frame helpers between PX4 (NED / FRD) and the ROS world frame used by the planner.
#ifndef PATH_PLANNING__PX4_FRAMES_HPP_
#define PATH_PLANNING__PX4_FRAMES_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace path_planning
{

// World frame convention used for the octomap / planning frame (z-up, right handed).
//  - ENU: x = East,  y = North, z = Up   (REP-103, px4_ros_com frame_transforms)
//  - NWU: x = North, y = West,  z = Up   (what the drone_pose package publishes: x, -y, -z)
// Both MUST match the convention used by whatever publishes the map -> base_link tf,
// otherwise the octomap and the planner positions will not line up.
enum class WorldFrame { ENU, NWU };

inline WorldFrame worldFrameFromString(const std::string & s)
{
  if (s == "enu" || s == "ENU") {return WorldFrame::ENU;}
  if (s == "nwu" || s == "NWU") {return WorldFrame::NWU;}
  throw std::invalid_argument("frame_convention must be 'enu' or 'nwu', got '" + s + "'");
}

inline Eigen::Vector3d nedToWorld(const Eigen::Vector3d & ned, WorldFrame f)
{
  if (f == WorldFrame::ENU) {return {ned.y(), ned.x(), -ned.z()};}
  return {ned.x(), -ned.y(), -ned.z()};
}

// Both mappings are involutions, so the inverse has the same form.
inline Eigen::Vector3d worldToNed(const Eigen::Vector3d & w, WorldFrame f)
{
  return nedToWorld(w, f);
}

inline double wrapPi(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Heading conversions (NED yaw: 0 = North, positive clockwise seen from above).
inline double nedYawToWorld(double yaw_ned, WorldFrame f)
{
  return wrapPi(f == WorldFrame::ENU ? M_PI_2 - yaw_ned : -yaw_ned);
}

inline double worldYawToNed(double yaw_w, WorldFrame f)
{
  return nedYawToWorld(yaw_w, f);
}

// Yaw from a PX4 quaternion [w, x, y, z] (FRD body w.r.t. NED).
inline double yawFromPx4Quaternion(const float q[4])
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace path_planning

#endif  // PATH_PLANNING__PX4_FRAMES_HPP_
