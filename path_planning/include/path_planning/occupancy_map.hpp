// Probabilistic 3D occupancy map built from lidar scans with Bonxai, and the immutable
// snapshots of its occupied voxels that the planner thread checks collisions against.
#ifndef PATH_PLANNING__OCCUPANCY_MAP_HPP_
#define PATH_PLANNING__OCCUPANCY_MAP_HPP_

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "bonxai/bonxai.hpp"

namespace Bonxai
{
class ProbabilisticMap;
}

namespace path_planning
{

struct MapOptions
{
  double resolution{0.2};  // voxel edge [m]
  double max_range{15.0};  // [m] longer rays only clear free space up to this distance
  double min_z{0.3};       // [m] points outside [min_z, max_z] in the map frame are dropped
  double max_z{6.0};
  double prob_hit{0.7};
  double prob_miss{0.4};
  double clamp_min{0.12};
  double clamp_max{0.97};
  bool exact_rays{true};   // exact voxel traversal (false: faster Bresenham approximation)
};

// Occupied voxels of the map at one instant. Read-only, so the planner thread can use it
// while the map keeps being updated.
class OccupancySnapshot
{
public:
  OccupancySnapshot(double resolution, const std::vector<Bonxai::CoordT> & occupied);

  double resolution() const {return resolution_;}
  size_t size() const {return voxels_.size();}
  const std::vector<Bonxai::CoordT> & voxels() const {return voxels_;}
  bool isOccupied(const Bonxai::CoordT & c) const;
  // True if any voxel with lo <= coord <= hi (inclusive, per axis) is occupied.
  bool anyOccupied(const Bonxai::CoordT & lo, const Bonxai::CoordT & hi) const;

  // Occupied voxel centres grouped in cubes of 8^3 voxels, for distance queries.
  struct Block
  {
    Eigen::Vector3d min, max;            // bounds of the voxels (not just their centres)
    std::vector<Eigen::Vector3d> centres;
  };
  const std::vector<Block> & blocks() const {return blocks_;}

private:
  double resolution_;
  Bonxai::VoxelGrid<uint8_t> grid_;
  std::vector<Bonxai::CoordT> voxels_;
  std::vector<Block> blocks_;
};

class OccupancyMap
{
public:
  explicit OccupancyMap(const MapOptions & options);
  ~OccupancyMap();

  // Ray-cast one scan into the map. Points and origin are in the map frame; non-finite
  // points and points outside [min_z, max_z] are skipped. Returns the points inserted.
  size_t insert(const std::vector<Eigen::Vector3d> & points, const Eigen::Vector3d & origin);
  void reset();

  std::shared_ptr<const OccupancySnapshot> snapshot() const;
  size_t memoryUsage() const;  // [bytes]
  const MapOptions & options() const {return options_;}

private:
  MapOptions options_;
  std::unique_ptr<Bonxai::ProbabilisticMap> map_;
  std::vector<Eigen::Vector3d> filtered_;
};

}  // namespace path_planning

#endif  // PATH_PLANNING__OCCUPANCY_MAP_HPP_
