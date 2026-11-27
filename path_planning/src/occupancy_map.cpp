#include "path_planning/occupancy_map.hpp"

#include <cmath>
#include <unordered_map>

#include "bonxai_map/probabilistic_map.hpp"

namespace path_planning
{

namespace
{

constexpr int kBlockBits = 3;  // distance-query blocks of 8^3 voxels

struct CoordHash
{
  size_t operator()(const Bonxai::CoordT & c) const
  {
    return (static_cast<size_t>(static_cast<uint32_t>(c.x)) * 73856093u) ^
           (static_cast<size_t>(static_cast<uint32_t>(c.y)) * 19349663u) ^
           (static_cast<size_t>(static_cast<uint32_t>(c.z)) * 83492791u);
  }
};

}  // namespace

// ============================================================================
// OccupancySnapshot
// ============================================================================

OccupancySnapshot::OccupancySnapshot(
  double resolution, const std::vector<Bonxai::CoordT> & occupied)
: resolution_(resolution), grid_(resolution), voxels_(occupied)
{
  auto accessor = grid_.createAccessor();
  std::unordered_map<Bonxai::CoordT, size_t, CoordHash> block_index;
  for (const auto & c : voxels_) {
    accessor.setValue(c, uint8_t{1});

    // >> on negative ints is an arithmetic (floor) shift since C++20
    const Bonxai::CoordT key{c.x >> kBlockBits, c.y >> kBlockBits, c.z >> kBlockBits};
    auto [it, inserted] = block_index.try_emplace(key, blocks_.size());
    if (inserted) {
      const double size = resolution_ * (1 << kBlockBits);
      Block b;
      b.min = Eigen::Vector3d(key.x, key.y, key.z) * size;
      b.max = b.min + Eigen::Vector3d::Constant(size);
      blocks_.push_back(std::move(b));
    }
    blocks_[it->second].centres.emplace_back(
      (c.x + 0.5) * resolution_, (c.y + 0.5) * resolution_, (c.z + 0.5) * resolution_);
  }
}

bool OccupancySnapshot::isOccupied(const Bonxai::CoordT & c) const
{
  return grid_.createConstAccessor().value(c) != nullptr;
}

bool OccupancySnapshot::anyOccupied(const Bonxai::CoordT & lo, const Bonxai::CoordT & hi) const
{
  // One accessor for the whole box: it caches the last leaf, and neighbours share leaves.
  auto accessor = grid_.createConstAccessor();
  for (int32_t x = lo.x; x <= hi.x; ++x) {
    for (int32_t y = lo.y; y <= hi.y; ++y) {
      for (int32_t z = lo.z; z <= hi.z; ++z) {
        if (accessor.value(Bonxai::CoordT{x, y, z}) != nullptr) {return true;}
      }
    }
  }
  return false;
}

// ============================================================================
// OccupancyMap
// ============================================================================

OccupancyMap::OccupancyMap(const MapOptions & options)
: options_(options)
{
  reset();
}

OccupancyMap::~OccupancyMap() = default;

void OccupancyMap::reset()
{
  using PM = Bonxai::ProbabilisticMap;
  map_ = std::make_unique<PM>(options_.resolution);
  PM::Options opt;
  opt.prob_hit_log = PM::logods(static_cast<float>(options_.prob_hit));
  opt.prob_miss_log = PM::logods(static_cast<float>(options_.prob_miss));
  opt.clamp_min_log = PM::logods(static_cast<float>(options_.clamp_min));
  opt.clamp_max_log = PM::logods(static_cast<float>(options_.clamp_max));
  opt.ray_mode = options_.exact_rays ? PM::Options::RayMode::Exact : PM::Options::RayMode::Approximate;
  map_->setOptions(opt);
}

size_t OccupancyMap::insert(
  const std::vector<Eigen::Vector3d> & points, const Eigen::Vector3d & origin)
{
  if (!origin.allFinite()) {return 0;}
  filtered_.clear();
  filtered_.reserve(points.size());
  for (const auto & p : points) {
    if (p.allFinite() && p.z() >= options_.min_z && p.z() <= options_.max_z) {
      filtered_.push_back(p);
    }
  }
  if (!filtered_.empty()) {map_->insertPointCloud(filtered_, origin, options_.max_range);}
  return filtered_.size();
}

std::shared_ptr<const OccupancySnapshot> OccupancyMap::snapshot() const
{
  std::vector<Bonxai::CoordT> occupied;
  map_->getOccupiedVoxels(occupied);
  return std::make_shared<const OccupancySnapshot>(options_.resolution, occupied);
}

size_t OccupancyMap::memoryUsage() const
{
  return map_->grid().memUsage();
}

}  // namespace path_planning
