#include "path_planning/global_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <utility>

namespace path_planning
{

namespace
{

// Lattice over the planning bounds, with lazily evaluated node validity.
class Lattice
{
public:
  Lattice(
    const CollisionWorld & world, const Eigen::Vector3d & min, const Eigen::Vector3d & max,
    double res)
  : world_(world), min_(min), res_(res)
  {
    for (int i = 0; i < 3; ++i) {
      n_[i] = std::max(1, static_cast<int>(std::floor((max[i] - min[i]) / res + 1e-9)) + 1);
    }
    valid_.assign(size(), kUnknown);
  }

  size_t size() const {return static_cast<size_t>(n_[0]) * n_[1] * n_[2];}
  bool inside(const Eigen::Vector3i & c) const
  {
    return (c.array() >= 0).all() && c.x() < n_[0] && c.y() < n_[1] && c.z() < n_[2];
  }
  int32_t index(const Eigen::Vector3i & c) const {return (c.z() * n_[1] + c.y()) * n_[0] + c.x();}
  Eigen::Vector3i cell(int32_t idx) const
  {
    return {idx % n_[0], (idx / n_[0]) % n_[1], idx / (n_[0] * n_[1])};
  }
  Eigen::Vector3d position(const Eigen::Vector3i & c) const {return min_ + c.cast<double>() * res_;}
  Eigen::Vector3i nearest(const Eigen::Vector3d & p) const
  {
    Eigen::Vector3i c;
    for (int i = 0; i < 3; ++i) {
      c[i] = std::clamp(static_cast<int>(std::lround((p[i] - min_[i]) / res_)), 0, n_[i] - 1);
    }
    return c;
  }
  bool valid(int32_t idx)
  {
    if (valid_[idx] == kUnknown) {
      valid_[idx] = world_.isFree(position(cell(idx))) ? kFree : kBlocked;
    }
    return valid_[idx] == kFree;
  }

  // Closest valid node within `radius` of p (ties: lowest index), if any.
  std::optional<Eigen::Vector3i> snap(const Eigen::Vector3d & p, double radius)
  {
    const Eigen::Vector3i c0 = nearest(p);
    if (valid(index(c0))) {return c0;}
    const int r = static_cast<int>(std::ceil(radius / res_));
    std::vector<std::pair<double, Eigen::Vector3i>> candidates;
    for (int dz = -r; dz <= r; ++dz) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          const Eigen::Vector3i c = c0 + Eigen::Vector3i(dx, dy, dz);
          if (!inside(c)) {continue;}
          const double d = (position(c) - p).norm();
          if (d <= radius) {candidates.emplace_back(d, c);}
        }
      }
    }
    std::stable_sort(
      candidates.begin(), candidates.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    for (const auto & [d, c] : candidates) {
      if (valid(index(c))) {return c;}
    }
    return std::nullopt;
  }

private:
  static constexpr uint8_t kUnknown = 0, kFree = 1, kBlocked = 2;
  const CollisionWorld & world_;
  Eigen::Vector3d min_;
  double res_;
  std::array<int, 3> n_{};
  std::vector<uint8_t> valid_;
};

}  // namespace

GlobalPlan GlobalPlanner::plan(
  const CollisionWorld & world, const Eigen::Vector3d & bounds_min,
  const Eigen::Vector3d & bounds_max, const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal) const
{
  GlobalPlan out;
  if (!start.allFinite() || !goal.allFinite()) {return out;}
  const double res = options_.resolution;
  Lattice lat(world, bounds_min, bounds_max, res);

  const auto s_cell = lat.snap(start, options_.snap_radius);
  const auto g_cell = lat.snap(goal, options_.snap_radius);
  if (!s_cell || !g_cell) {return out;}
  const int32_t s_idx = lat.index(*s_cell), g_idx = lat.index(*g_cell);
  const Eigen::Vector3d g_pos = lat.position(*g_cell);

  // A diagonal move sweeps the box across the cube spanned by its two nodes. When the
  // lattice step is at most the box size, the boxes at the cube's corners cover that whole
  // volume, so the move is collision free exactly when every corner node is valid (no corner
  // cutting). `corners` lists the corners other than the two end nodes.
  struct Move
  {
    Eigen::Vector3i d;
    float cost;
    std::vector<Eigen::Vector3i> corners;
  };
  std::vector<Move> moves;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy && !dz) {continue;}
        Move m{{dx, dy, dz}, static_cast<float>(res * std::sqrt(dx * dx + dy * dy + dz * dz)), {}};
        for (int mask = 1; mask < 7; ++mask) {
          const Eigen::Vector3i e((mask & 1) ? dx : 0, (mask & 2) ? dy : 0, (mask & 4) ? dz : 0);
          if (e != Eigen::Vector3i::Zero() && e != m.d &&
            std::find(m.corners.begin(), m.corners.end(), e) == m.corners.end())
          {
            m.corners.push_back(e);
          }
        }
        moves.push_back(std::move(m));
      }
    }
  }

  // A*: a slightly inflated heuristic breaks ties towards the goal (1e-3 longer paths at most).
  constexpr float kTieBreak = 1.001f;
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> g(lat.size(), inf);
  std::vector<int32_t> parent(lat.size(), -1);
  std::vector<uint8_t> closed(lat.size(), 0);
  using Entry = std::pair<float, int32_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  auto h = [&](const Eigen::Vector3i & c) {
      return kTieBreak * static_cast<float>((lat.position(c) - g_pos).norm());
    };
  g[s_idx] = 0.0f;
  open.emplace(h(*s_cell), s_idx);

  bool found = false;
  while (!open.empty()) {
    const auto [f, idx] = open.top();
    open.pop();
    if (closed[idx]) {continue;}
    closed[idx] = 1;
    if (++out.expansions > options_.max_expansions) {break;}
    if (idx == g_idx) {found = true; break;}
    const Eigen::Vector3i c = lat.cell(idx);
    for (const auto & m : moves) {
      const Eigen::Vector3i nc = c + m.d;
      if (!lat.inside(nc)) {continue;}
      const int32_t nidx = lat.index(nc);
      if (closed[nidx] || !lat.valid(nidx)) {continue;}
      bool swept_free = true;
      for (const auto & e : m.corners) {
        const Eigen::Vector3i cc = c + e;
        if (!lat.inside(cc) || !lat.valid(lat.index(cc))) {swept_free = false; break;}
      }
      if (!swept_free) {continue;}
      const float ng = g[idx] + m.cost;
      if (ng < g[nidx]) {
        g[nidx] = ng;
        parent[nidx] = idx;
        open.emplace(ng + h(nc), nidx);
      }
    }
  }
  if (!found) {return out;}

  // Node chain start -> goal, with the exact start (the drone is there) and the exact goal
  // when it is itself a valid position.
  std::vector<Eigen::Vector3d> chain;
  for (int32_t idx = g_idx; idx != -1; idx = parent[idx]) {chain.push_back(lat.position(lat.cell(idx)));}
  std::reverse(chain.begin(), chain.end());
  if ((chain.front() - start).norm() > 1e-9) {chain.insert(chain.begin(), start);}
  if (world.isFree(goal) && (chain.back() - goal).norm() > 1e-9) {chain.push_back(goal);}

  // Line-of-sight shortcutting: from each kept point jump to the farthest visible one.
  out.path.push_back(chain.front());
  size_t i = 0;
  while (i + 1 < chain.size()) {
    size_t next = i + 1;
    for (size_t j = chain.size() - 1; j > i + 1; --j) {
      if (world.isSegmentFree(chain[i], chain[j], options_.shortcut_step)) {next = j; break;}
    }
    out.path.push_back(chain[next]);
    i = next;
  }
  out.length = polylineLength(out.path);
  out.success = true;
  return out;
}

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

}  // namespace path_planning
