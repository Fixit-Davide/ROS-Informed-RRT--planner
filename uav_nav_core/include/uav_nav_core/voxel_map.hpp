// The map as it travels between the navigation servers.
//
// VoxelMap is a header plus a shared pointer to an immutable OccupancySnapshot. The
// rclcpp::TypeAdapter below lets nodes publish and subscribe it directly on a
// uav_nav_msgs/msg/VoxelMap topic:
//
//  - inside one process with intra-process communication enabled (the servers loaded in one
//    component container), the published object itself reaches every subscriber: no
//    serialization, no ROS message, no copy of the voxels;
//  - subscribers in other processes (ros2 topic echo, a node run on its own) get the ROS
//    message, built with convert_to_ros_message() only when such a subscriber exists.
//
// Intra-process needs a volatile, keep-last publisher: with transient-local durability
// rclcpp converts every message to keep it for late joiners, which defeats the point.
// The snapshot is immutable, so sharing it between threads and nodes needs no lock.
#ifndef UAV_NAV_CORE__VOXEL_MAP_HPP_
#define UAV_NAV_CORE__VOXEL_MAP_HPP_

#include <memory>
#include <type_traits>

#include <rclcpp/qos.hpp>
#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/header.hpp>
#include <uav_nav_msgs/msg/voxel_map.hpp>

#include "uav_nav_core/occupancy_map.hpp"

namespace uav_nav_core
{

struct VoxelMap
{
  std_msgs::msg::Header header;                       // map frame, stamp of the last scan
  std::shared_ptr<const OccupancySnapshot> snapshot;  // never null once published
};

void toMsg(const VoxelMap & map, uav_nav_msgs::msg::VoxelMap & msg);
// Rebuilds the snapshot (voxel grid and distance blocks) from the cell list.
void fromMsg(const uav_nav_msgs::msg::VoxelMap & msg, VoxelMap & map);

// Volatile, keep last 1: maps are republished with every scan, a late joiner gets the next one.
inline rclcpp::QoS voxelMapQoS() {return rclcpp::QoS(rclcpp::KeepLast(1)).reliable();}

}  // namespace uav_nav_core

template<>
struct rclcpp::TypeAdapter<uav_nav_core::VoxelMap, uav_nav_msgs::msg::VoxelMap>
{
  using is_specialized = std::true_type;
  using custom_type = uav_nav_core::VoxelMap;
  using ros_message_type = uav_nav_msgs::msg::VoxelMap;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    uav_nav_core::toMsg(source, destination);
  }
  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    uav_nav_core::fromMsg(source, destination);
  }
};

// create_publisher<uav_nav_core::VoxelMap>() / create_subscription<uav_nav_core::VoxelMap>()
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(uav_nav_core::VoxelMap, uav_nav_msgs::msg::VoxelMap);

#endif  // UAV_NAV_CORE__VOXEL_MAP_HPP_
