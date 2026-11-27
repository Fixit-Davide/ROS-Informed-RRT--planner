// octomap (as octomap_server + FCL use it) vs Bonxai ProbabilisticMap, on recorded SITL lidar scans.
//
// Mapping: every scan is inserted into both maps with the same sensor model and max range.
//   octomap: the exact insertScan() logic of octomap_server (KeySets, updateNode, prune),
//            then what publishAll() + the planner do per scan: iterate leaves for occupied
//            cells, serialise the binary map, deserialise it.
//   Bonxai : ProbabilisticMap::insertPointCloud, then getOccupiedVoxels (bonxai_server's
//            published cloud), Serialize/Deserialize of the grid.
// Collision: the planner's check (robot box at a point vs occupied voxels):
//   FCL box vs fcl::OcTree (what CollisionWorld does now), a voxel loop over the octree,
//   and the same voxel loop over Bonxai.
//
// usage: bench scans.bin [resolution=0.2] [max_range=15] [ray=exact|approx]

#include <octomap/OcTree.h>

#include <fcl/fcl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "bonxai/serialization.hpp"
#include "bonxai_map/probabilistic_map.hpp"

using Clock = std::chrono::steady_clock;
static double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Scan
{
  Eigen::Vector3d origin;
  std::vector<Eigen::Vector3d> pts;
};

static std::vector<Scan> load(const char * path)
{
  std::ifstream f(path, std::ios::binary);
  std::vector<Scan> scans;
  uint32_t n;
  while (f.read(reinterpret_cast<char *>(&n), 4)) {
    float o[3];
    f.read(reinterpret_cast<char *>(o), 12);
    std::vector<float> xyz(n * 3);
    f.read(reinterpret_cast<char *>(xyz.data()), n * 12);
    Scan s;
    s.origin = {o[0], o[1], o[2]};
    s.pts.reserve(n);
    for (uint32_t i = 0; i < n; ++i) {s.pts.emplace_back(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);}
    scans.push_back(std::move(s));
  }
  return scans;
}

struct Stats
{
  std::vector<double> v;
  void add(double x) {v.push_back(x);}
  double mean() const {double s = 0; for (double x : v) {s += x;} return v.empty() ? 0 : s / v.size();}
  double pct(double p) const
  {
    if (v.empty()) {return 0;}
    auto c = v;
    std::sort(c.begin(), c.end());
    return c[std::min(c.size() - 1, static_cast<size_t>(p * c.size()))];
  }
  double sum() const {double s = 0; for (double x : v) {s += x;} return s;}
};

static void row(const char * name, const Stats & s)
{
  std::printf("  %-34s mean %8.3f  p50 %8.3f  p95 %8.3f  max %8.3f  total %9.1f ms\n",
    name, s.mean(), s.pct(0.5), s.pct(0.95), s.pct(1.0), s.sum());
}

// ---------------------------------------------------------------- octomap_server insertScan
static void octomapInsert(octomap::OcTree & tree, const Scan & s, double max_range)
{
  const octomap::point3d origin(s.origin.x(), s.origin.y(), s.origin.z());
  octomap::KeySet free_cells, occupied_cells;
  octomap::KeyRay ray;
  for (const auto & p : s.pts) {
    octomap::point3d point(p.x(), p.y(), p.z());
    if (max_range < 0.0 || (point - origin).norm() <= max_range) {
      if (tree.computeRayKeys(origin, point, ray)) {free_cells.insert(ray.begin(), ray.end());}
      octomap::OcTreeKey key;
      if (tree.coordToKeyChecked(point, key)) {occupied_cells.insert(key);}
    } else {
      const octomap::point3d new_end = origin + (point - origin).normalized() * max_range;
      if (tree.computeRayKeys(origin, new_end, ray)) {
        free_cells.insert(ray.begin(), ray.end());
        octomap::OcTreeKey end_key;
        if (tree.coordToKeyChecked(new_end, end_key)) {free_cells.insert(end_key);}
      }
    }
  }
  for (const auto & k : free_cells) {
    if (occupied_cells.find(k) == occupied_cells.end()) {tree.updateNode(k, false);}
  }
  for (const auto & k : occupied_cells) {tree.updateNode(k, true);}
  tree.prune();
}

// ---------------------------------------------------------------- collision checks
// Voxel index range overlapped by the box [c - h, c + h] (touching faces excluded).
static void boxRange(const Eigen::Vector3d & c, const Eigen::Vector3d & h, double res,
  int lo[3], int hi[3])
{
  for (int i = 0; i < 3; ++i) {
    lo[i] = static_cast<int>(std::floor((c[i] - h[i]) / res + 1e-9));
    hi[i] = static_cast<int>(std::ceil((c[i] + h[i]) / res - 1e-9)) - 1;
  }
}

static bool octreeLoopFree(const octomap::OcTree & t, const Eigen::Vector3d & c,
  const Eigen::Vector3d & h, double res)
{
  int lo[3], hi[3];
  boxRange(c, h, res, lo, hi);
  for (int x = lo[0]; x <= hi[0]; ++x) {
    for (int y = lo[1]; y <= hi[1]; ++y) {
      for (int z = lo[2]; z <= hi[2]; ++z) {
        const auto * n = t.search((x + 0.5) * res, (y + 0.5) * res, (z + 0.5) * res);
        if (n && t.isNodeOccupied(n)) {return false;}
      }
    }
  }
  return true;
}

static bool bonxaiLoopFree(const Bonxai::ProbabilisticMap & m, const Eigen::Vector3d & c,
  const Eigen::Vector3d & h, double res)
{
  int lo[3], hi[3];
  boxRange(c, h, res, lo, hi);
  for (int x = lo[0]; x <= hi[0]; ++x) {
    for (int y = lo[1]; y <= hi[1]; ++y) {
      for (int z = lo[2]; z <= hi[2]; ++z) {
        if (m.isOccupied(Bonxai::CoordT{x, y, z})) {return false;}
      }
    }
  }
  return true;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {std::fprintf(stderr, "usage: bench scans.bin [res] [max_range] [exact|approx]\n"); return 1;}
  const double res = argc > 2 ? std::atof(argv[2]) : 0.2;
  const double max_range = argc > 3 ? std::atof(argv[3]) : 15.0;
  const bool exact = argc > 4 ? std::string(argv[4]) != "approx" : true;

  const auto scans = load(argv[1]);
  size_t npts = 0;
  for (const auto & s : scans) {npts += s.pts.size();}
  std::printf("%zu scans, %.0f points/scan, resolution %.2f m, max_range %.1f m, Bonxai rays %s\n",
    scans.size(), double(npts) / scans.size(), res, max_range, exact ? "exact" : "approximate");

  // Same sensor model as octomap_server defaults (hit 0.7, miss 0.4, clamp 0.12/0.97).
  octomap::OcTree tree(res);
  tree.setProbHit(0.7);
  tree.setProbMiss(0.4);
  tree.setClampingThresMin(0.12);
  tree.setClampingThresMax(0.97);
  tree.setOccupancyThres(0.5);

  Bonxai::ProbabilisticMap bmap(res);
  auto opt = bmap.options();
  opt.ray_mode = exact ? Bonxai::ProbabilisticMap::Options::RayMode::Exact :
    Bonxai::ProbabilisticMap::Options::RayMode::Approximate;
  bmap.setOptions(opt);

  Stats o_ins, o_iter, o_ser, o_deser, b_ins, b_iter, b_ser, b_deser, o_bytes, b_bytes;
  std::vector<std::tuple<size_t, size_t, size_t>> mem_curve;  // scan, octomap B, bonxai B
  const size_t every = 10;  // publish / serialise cost sampled every 10th scan (it grows with the map)

  for (size_t i = 0; i < scans.size(); ++i) {
    const auto & s = scans[i];
    auto t0 = Clock::now();
    octomapInsert(tree, s, max_range);
    o_ins.add(ms_since(t0));

    t0 = Clock::now();
    bmap.insertPointCloud(s.pts, s.origin, max_range);
    b_ins.add(ms_since(t0));

    if (i % every == 0 || i + 1 == scans.size()) {
      // octomap_server publishAll(): walk the leaves, keep occupied centres (markers / cloud)
      t0 = Clock::now();
      std::vector<octomap::point3d> occ;
      for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {occ.push_back(it.getCoordinate());}
      }
      o_iter.add(ms_since(t0));
      // /octomap_binary: writeBinary on the server, readBinary in the planner
      t0 = Clock::now();
      std::stringstream ss;
      tree.writeBinaryData(ss);
      o_ser.add(ms_since(t0));
      o_bytes.add(ss.str().size());
      t0 = Clock::now();
      octomap::OcTree copy(res);
      copy.readBinaryData(ss);
      o_deser.add(ms_since(t0));

      // bonxai_server: occupied voxel centres as a PointCloud2
      t0 = Clock::now();
      std::vector<Eigen::Vector3d> bocc;
      bmap.getOccupiedVoxels(bocc);
      b_iter.add(ms_since(t0));
      t0 = Clock::now();
      std::stringstream bs;
      Bonxai::Serialize(bs, bmap.grid());
      b_ser.add(ms_since(t0));
      b_bytes.add(bs.str().size());
      t0 = Clock::now();
      char header[256];
      bs.getline(header, 256);
      auto info = Bonxai::GetHeaderInfo(header);
      auto grid = Bonxai::Deserialize<Bonxai::ProbabilisticMap::CellT>(bs, info);
      b_deser.add(ms_since(t0));

      mem_curve.emplace_back(i, tree.memoryUsage(), bmap.grid().memUsage());
    }
  }

  std::printf("\nMap update per scan [ms]\n");
  row("octomap insertScan + prune", o_ins);
  row("Bonxai insertPointCloud", b_ins);
  std::printf("  speed-up %.2fx\n", o_ins.mean() / b_ins.mean());

  std::printf("\nPublishing / transport, sampled every %zu scans [ms]\n", every);
  row("octomap leaf walk (occupied)", o_iter);
  row("Bonxai getOccupiedVoxels", b_iter);
  row("octomap writeBinary", o_ser);
  row("octomap readBinary (planner)", o_deser);
  row("Bonxai Serialize", b_ser);
  row("Bonxai Deserialize", b_deser);
  std::printf("  final message size: octomap binary %.1f kB, Bonxai serialised %.1f kB\n",
    o_bytes.v.back() / 1024.0, b_bytes.v.back() / 1024.0);

  std::printf("\nMemory (library's own accounting) [kB]\n  scan   octomap    Bonxai\n");
  for (size_t k = 0; k < mem_curve.size(); k += std::max<size_t>(1, mem_curve.size() / 8)) {
    auto [i, om, bm] = mem_curve[k];
    std::printf("  %4zu %9.1f %9.1f\n", i, om / 1024.0, bm / 1024.0);
  }
  auto [li, lom, lbm] = mem_curve.back();
  std::printf("  %4zu %9.1f %9.1f   (final)\n", li, lom / 1024.0, lbm / 1024.0);

  // ---------------------------------------------------------------- map agreement
  std::set<std::tuple<int, int, int>> o_occ, b_occ;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (!tree.isNodeOccupied(*it)) {continue;}
    // pruned leaves can be bigger than one voxel: expand them
    const double sz = it.getSize();
    const int n = static_cast<int>(std::lround(sz / res));
    const auto c = it.getCoordinate();
    for (int dx = 0; dx < n; ++dx) {
      for (int dy = 0; dy < n; ++dy) {
        for (int dz = 0; dz < n; ++dz) {
          o_occ.emplace(
            static_cast<int>(std::floor((c.x() - sz / 2) / res + 1e-6)) + dx,
            static_cast<int>(std::floor((c.y() - sz / 2) / res + 1e-6)) + dy,
            static_cast<int>(std::floor((c.z() - sz / 2) / res + 1e-6)) + dz);
        }
      }
    }
  }
  std::vector<Bonxai::CoordT> bc;
  bmap.getOccupiedVoxels(bc);
  for (const auto & c : bc) {b_occ.emplace(c.x, c.y, c.z);}
  size_t inter = 0;
  for (const auto & k : o_occ) {inter += b_occ.count(k);}
  std::printf("\nOccupied voxels: octomap %zu, Bonxai %zu, both %zu, IoU %.3f\n",
    o_occ.size(), b_occ.size(), inter, double(inter) / (o_occ.size() + b_occ.size() - inter));

  // ---------------------------------------------------------------- collision checks
  // Planner config: robot box 0.90 x 0.90 x 0.60 + planning margin 0.15 per side.
  const Eigen::Vector3d half(0.6, 0.6, 0.45);
  auto box = std::make_shared<fcl::Boxd>(2 * half.x(), 2 * half.y(), 2 * half.z());
  std::shared_ptr<const octomap::OcTree> shared(new octomap::OcTree(tree));
  fcl::CollisionObjectd tree_obj(std::make_shared<fcl::OcTreed>(shared));

  // Sample states inside the planner bounds near the explored area.
  double mn[3] = {1e9, 1e9, 1e9}, mx[3] = {-1e9, -1e9, -1e9};
  for (const auto & s : scans) {
    for (int i = 0; i < 3; ++i) {mn[i] = std::min(mn[i], s.origin[i]); mx[i] = std::max(mx[i], s.origin[i]);}
  }
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(mn[0] - 4, mx[0] + 4), uy(mn[1] - 4, mx[1] + 4), uz(0.5, 4.0);
  const int N = 200000;
  std::vector<Eigen::Vector3d> states(N);
  for (auto & p : states) {p = {ux(rng), uy(rng), uz(rng)};}

  std::vector<char> r_fcl(N), r_oloop(N), r_bloop(N);
  auto t0 = Clock::now();
  for (int i = 0; i < N; ++i) {
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    tf.translation() = states[i];
    fcl::CollisionObjectd robot(box, tf);
    fcl::CollisionRequestd req(1, false);
    fcl::CollisionResultd resu;
    fcl::collide(&robot, &tree_obj, req, resu);
    r_fcl[i] = !resu.isCollision();
  }
  const double t_fcl = ms_since(t0);
  t0 = Clock::now();
  for (int i = 0; i < N; ++i) {r_oloop[i] = octreeLoopFree(*shared, states[i], half, res);}
  const double t_oloop = ms_since(t0);
  t0 = Clock::now();
  for (int i = 0; i < N; ++i) {r_bloop[i] = bonxaiLoopFree(bmap, states[i], half, res);}
  const double t_bloop = ms_since(t0);

  int free_fcl = 0, d_loop = 0, d_lib = 0, only_b_free = 0;
  for (int i = 0; i < N; ++i) {
    free_fcl += r_fcl[i];
    d_loop += r_fcl[i] != r_oloop[i];
    d_lib += r_oloop[i] != r_bloop[i];
    only_b_free += r_bloop[i] && !r_oloop[i];
  }
  std::printf("\nCollision check, robot box %.2fx%.2fx%.2f m, %d random states (%.1f%% free by FCL)\n",
    2 * half.x(), 2 * half.y(), 2 * half.z(), N, 100.0 * free_fcl / N);
  std::printf("  FCL box vs OcTree (current)        %7.3f us/check\n", 1e3 * t_fcl / N);
  std::printf("  voxel loop over octomap            %7.3f us/check  (disagrees with FCL on %d)\n",
    1e3 * t_oloop / N, d_loop);
  std::printf("  voxel loop over Bonxai             %7.3f us/check  (disagrees with octomap loop on %d = %.2f%%)\n",
    1e3 * t_bloop / N, d_lib, 100.0 * d_lib / N);
  std::printf("    of which free only in Bonxai %d, free only in octomap %d\n", only_b_free, d_lib - only_b_free);
  return 0;
}
