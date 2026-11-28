// Map server: builds the 3D occupancy map (Bonxai) from the lidar and publishes a snapshot of
// its occupied voxels after every scan as uav_nav_core::VoxelMap. Loaded in the same
// component container as the planner and controller servers (intra-process comms on), they
// receive the published snapshot itself: the map is never copied or serialized.
#ifndef UAV_MAP_SERVER__MAP_SERVER_HPP_
#define UAV_MAP_SERVER__MAP_SERVER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <message_filters/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "uav_nav_core/occupancy_map.hpp"
#include "uav_nav_core/voxel_map.hpp"

namespace uav_map_server
{

class MapServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapServer() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  // Snapshot of the map -> voxel_map topic (and the RViz markers, throttled).
  void publishMap(const rclcpp::Time & stamp);
  void publishMarkers(const uav_nav_core::OccupancySnapshot & snap, const rclcpp::Time & stamp);
  void resetMap();

  // parameters
  std::string map_frame_, cloud_topic_, map_topic_;
  uav_nav_core::MapOptions map_options_;
  double marker_period_{0.5};    // [s] (0 = no markers)
  double republish_period_{1.0}; // [s] republish the last snapshot when no scan arrives
  double marker_max_z_{5.0};     // [m] top of the marker colour scale

  std::unique_ptr<uav_nav_core::OccupancyMap> map_;
  std::shared_ptr<const uav_nav_core::OccupancySnapshot> last_snapshot_;
  std::vector<Eigen::Vector3d> cloud_points_;

  rclcpp::Publisher<uav_nav_core::VoxelMap>::SharedPtr map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;

  bool active_{false};
  std::optional<rclcpp::Time> last_marker_publish_;
  bool published_since_tick_{false};
  double insert_ms_sum_{0.0}, snapshot_ms_sum_{0.0};
  size_t scans_since_log_{0}, scans_total_{0};
};

}  // namespace uav_map_server

#endif  // UAV_MAP_SERVER__MAP_SERVER_HPP_
