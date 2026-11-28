#include "uav_map_server/map_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/create_timer_ros.hpp>

using namespace std::chrono_literals;

namespace uav_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("map_server", options)
{
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("topics.cloud", "/lidar/points");
  declare_parameter<std::string>("topics.map", "voxel_map");
  declare_parameter<double>("resolution", 0.2);
  declare_parameter<double>("max_range", 15.0);
  declare_parameter<double>("min_z", 0.3);
  declare_parameter<double>("max_z", 6.0);
  declare_parameter<double>("sensor_model.hit", 0.7);
  declare_parameter<double>("sensor_model.miss", 0.4);
  declare_parameter<double>("sensor_model.min", 0.12);
  declare_parameter<double>("sensor_model.max", 0.97);
  declare_parameter<bool>("exact_rays", true);
  declare_parameter<double>("republish_period", 1.0);
  declare_parameter<double>("markers.publish_rate_hz", 2.0);
  declare_parameter<double>("markers.max_z", 5.0);
}

MapServer::~MapServer() = default;

MapServer::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State &)
{
  map_frame_ = get_parameter("map_frame").as_string();
  cloud_topic_ = get_parameter("topics.cloud").as_string();
  map_topic_ = get_parameter("topics.map").as_string();
  map_options_.resolution = get_parameter("resolution").as_double();
  map_options_.max_range = get_parameter("max_range").as_double();
  map_options_.min_z = get_parameter("min_z").as_double();
  map_options_.max_z = get_parameter("max_z").as_double();
  map_options_.prob_hit = get_parameter("sensor_model.hit").as_double();
  map_options_.prob_miss = get_parameter("sensor_model.miss").as_double();
  map_options_.clamp_min = get_parameter("sensor_model.min").as_double();
  map_options_.clamp_max = get_parameter("sensor_model.max").as_double();
  map_options_.exact_rays = get_parameter("exact_rays").as_bool();
  republish_period_ = get_parameter("republish_period").as_double();
  const double marker_rate = get_parameter("markers.publish_rate_hz").as_double();
  marker_period_ = marker_rate > 0.0 ? 1.0 / marker_rate : 0.0;
  marker_max_z_ = get_parameter("markers.max_z").as_double();
  if (!(map_options_.resolution > 0.0)) {
    RCLCPP_ERROR(get_logger(), "resolution must be > 0");
    return CallbackReturn::FAILURE;
  }

  map_ = std::make_unique<uav_nav_core::OccupancyMap>(map_options_);
  last_snapshot_ = map_->snapshot();

  // A plain rclcpp publisher: LifecyclePublisher has no type-adapted publish(unique_ptr).
  // publishMap() only runs while active.
  map_pub_ = rclcpp::create_publisher<uav_nav_core::VoxelMap>(
    *this, map_topic_, uav_nav_core::voxelMapQoS());
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/occupied_voxels", rclcpp::QoS(1).transient_local());
  reset_srv_ = create_service<std_srvs::srv::Empty>(
    "~/reset_map",
    [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {resetMap();});

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(
    get_logger(), "Configured: %.2f m voxels, %s rays, cloud %s -> %s", map_options_.resolution,
    map_options_.exact_rays ? "exact" : "approximate", cloud_topic_.c_str(), map_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State &)
{
  marker_pub_->on_activate();
  active_ = true;
  // Scans are held back until map_frame -> cloud frame is known at their stamp.
  cloud_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
  cloud_sub_->subscribe(this, cloud_topic_, rclcpp::SensorDataQoS());
  cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *cloud_sub_, *tf_buffer_, map_frame_, 5, get_node_logging_interface(),
    get_node_clock_interface(), 1s);
  cloud_filter_->registerCallback(&MapServer::cloudCallback, this);
  if (republish_period_ > 0.0) {
    // Without scans (lidar off, map just reset) subscribers still get a map regularly.
    republish_timer_ = create_wall_timer(
      std::chrono::duration<double>(republish_period_), [this]() {
        if (!published_since_tick_) {publishMap(now());}
        published_since_tick_ = false;
      });
  }
  publishMap(now());
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  republish_timer_.reset();
  cloud_filter_.reset();
  cloud_sub_.reset();
  marker_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  tf_listener_.reset();
  tf_buffer_.reset();
  reset_srv_.reset();
  marker_pub_.reset();
  map_pub_.reset();
  last_snapshot_.reset();
  map_.reset();
  last_marker_publish_.reset();
  return CallbackReturn::SUCCESS;
}

MapServer::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (active_) {on_deactivate(state);}
  return on_cleanup(state);
}

void MapServer::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (!active_ || !map_) {return;}
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping scan: %s", e.what());
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();

  const auto & tr = tf.transform.translation;
  const auto & q = tf.transform.rotation;
  const Eigen::Matrix3d rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  const Eigen::Vector3d origin(tr.x, tr.y, tr.z);

  cloud_points_.clear();
  cloud_points_.reserve(static_cast<size_t>(msg->width) * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
    cloud_points_.push_back(rot * Eigen::Vector3d(*x, *y, *z) + origin);
  }
  map_->insert(cloud_points_, origin);
  const auto t1 = std::chrono::steady_clock::now();
  publishMap(msg->header.stamp);
  const auto t2 = std::chrono::steady_clock::now();

  insert_ms_sum_ += std::chrono::duration<double, std::milli>(t1 - t0).count();
  snapshot_ms_sum_ += std::chrono::duration<double, std::milli>(t2 - t1).count();
  ++scans_since_log_;
  ++scans_total_;
  if (scans_since_log_ >= 100) {
    const double n = static_cast<double>(scans_since_log_);
    RCLCPP_INFO(
      get_logger(), "Map: %zu scans, %zu occupied voxels, %.2f MB; per scan %.2f ms insert + "
      "%.2f ms snapshot and publish (%zu intra-process / %zu total subscribers)", scans_total_,
      last_snapshot_->size(), static_cast<double>(map_->memoryUsage()) / (1024.0 * 1024.0),
      insert_ms_sum_ / n, snapshot_ms_sum_ / n, map_pub_->get_intra_process_subscription_count(),
      map_pub_->get_subscription_count());
    insert_ms_sum_ = snapshot_ms_sum_ = 0.0;
    scans_since_log_ = 0;
  }
}

void MapServer::publishMap(const rclcpp::Time & stamp)
{
  if (!active_ || !map_) {return;}
  last_snapshot_ = map_->snapshot();
  auto msg = std::make_unique<uav_nav_core::VoxelMap>();
  msg->header.frame_id = map_frame_;
  msg->header.stamp = stamp;
  msg->snapshot = last_snapshot_;
  map_pub_->publish(std::move(msg));
  published_since_tick_ = true;

  const auto t = now();
  if (marker_period_ > 0.0 && marker_pub_->get_subscription_count() > 0 &&
    (!last_marker_publish_ || (t - *last_marker_publish_).seconds() >= marker_period_))
  {
    last_marker_publish_ = t;
    publishMarkers(*last_snapshot_, t);
  }
}

void MapServer::publishMarkers(
  const uav_nav_core::OccupancySnapshot & snap, const rclcpp::Time & stamp)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_frame_;
  m.header.stamp = stamp;
  m.ns = "occupied";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  const double res = snap.resolution();
  m.scale.x = m.scale.y = m.scale.z = res;
  m.points.reserve(snap.size());
  m.colors.reserve(snap.size());
  // Colour by height, like octomap_server's height map: blue (low) -> red (high).
  const double z_lo = map_options_.min_z, z_hi = marker_max_z_;
  for (const auto & c : snap.voxels()) {
    geometry_msgs::msg::Point p;
    p.x = (c.x + 0.5) * res;
    p.y = (c.y + 0.5) * res;
    p.z = (c.z + 0.5) * res;
    m.points.push_back(p);
    const double h = std::clamp((p.z - z_lo) / std::max(z_hi - z_lo, 1e-3), 0.0, 1.0);
    std_msgs::msg::ColorRGBA col;
    col.r = static_cast<float>(std::clamp(2.0 * h - 0.5, 0.0, 1.0));
    col.g = static_cast<float>(1.0 - std::abs(2.0 * h - 1.0));
    col.b = static_cast<float>(std::clamp(1.5 - 2.0 * h, 0.0, 1.0));
    col.a = 1.0f;
    m.colors.push_back(col);
  }
  if (m.points.empty()) {m.action = visualization_msgs::msg::Marker::DELETE;}
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(std::move(m));
  marker_pub_->publish(arr);
}

void MapServer::resetMap()
{
  if (!map_) {return;}
  map_->reset();
  last_marker_publish_.reset();
  publishMap(now());
  RCLCPP_INFO(get_logger(), "Map reset");
}

}  // namespace uav_map_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uav_map_server::MapServer)
