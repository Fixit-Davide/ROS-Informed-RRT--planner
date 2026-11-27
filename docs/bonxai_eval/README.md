# Bonxai vs octomap for the planner's map

This evaluation asks whether [Bonxai](https://github.com/facontidavide/Bonxai) should replace octomap in the mapping half of this repository. Today octomap_server builds the map from the lidar and the planner checks collisions on it through FCL. All numbers below come from replaying the same recorded lidar scans through both libraries.

## Setup

- **Data**: 1,299 scans of the 16-channel `x500_lidar_3d` lidar, recorded at 10 Hz during the usual walls-world tour flown by the planner. The goals were (-8,6,2), (0,0,2), (0,-8,2), (0,0,2), about 87 m in all. That is 3,912 points per scan after the same filter as `config/mapping.yaml` (finite points, z in [0.3, 6.0] m in `map`). `capture_scans.py` stores each scan in the map frame with its sensor origin.
- **Same map model** on both sides: resolution 0.2 m (the current setting), max range 15 m, hit 0.7, miss 0.4, clamping 0.12/0.97, occupancy threshold 0.5.
- **octomap**: `bench.cpp` copies `OctomapServer::insertScan()` (free and occupied KeySets, `updateNode`, `prune`). It then times what happens to every scan after insertion:
  - the occupied-leaf walk that `publishAll()` does for the markers and the cloud;
  - `writeBinary` for `/octomap_binary`;
  - `readBinary`, which the planner runs on every message.
- **Bonxai** at commit 8ba04b2:
  - `ProbabilisticMap::insertPointCloud`, the same call `bonxai_server` makes;
  - `getOccupiedVoxels`, which is what `bonxai_server` publishes;
  - `Serialize` and `Deserialize` of the grid.
- **Collision check**: the planner's box, 0.90 × 0.90 × 0.60 m plus a 0.15 m planning margin per side (1.20 × 1.20 × 0.90 m), at 200,000 random states around the explored area. Three variants:
  - FCL box against `fcl::OcTree`, which is what `CollisionWorld::isFree` does now;
  - a plain loop over the voxels under the box, on the octree;
  - the same loop on Bonxai.

  The octree loop agrees with FCL on all 200,000 states, which confirms the loop is the same test.
- **Machine and build**: Xeon E5-2620 v3, one thread, `-O2 -march=native` like the workspace build. The distro packages were octomap 1.9.7 and FCL 0.7.0. SITL was running in the background; repeated runs differ by less than 1 %.

## Results

### Map update (per scan, 0.2 m)

| | mean | p95 | max |
|---|---|---|---|
| octomap `insertScan` + prune | 13.4 ms | 19.2 ms | 28.0 ms |
| Bonxai `insertPointCloud` (exact rays) | 1.52 ms | 2.15 ms | 2.7 ms |
| Bonxai `insertPointCloud` (approximate rays) | 1.25 ms | 1.81 ms | 3.1 ms |

Bonxai inserts **8.8× faster** with exact rays and 10.4× with approximate rays.

### Publishing and transport (full map at the end of the tour, 0.2 m)

| | octomap | Bonxai |
|---|---|---|
| Collect occupied voxels for RViz | 3.9 ms (leaf walk) | 1.4 ms (`getOccupiedVoxels`) |
| Serialise the map | 2.4 ms, **27 kB** | 5.3 ms, **830 kB** |
| Deserialise in the planner | 5.6 ms | 4.7 ms |

octomap's binary format stores 2 bits per node of a pruned tree, so it is about 31× smaller than Bonxai's serialised grid. Bonxai writes the full 4-byte cell, free cells included.

Shipping Bonxai's serialised grid to the planner every scan would be about 8 MB/s at 10 Hz. Bonxai therefore only pays off if the map stays in the planner's process, or if only the occupied voxels are sent, as a cloud or a bit mask.

### Whole mapping pipeline per scan

Each sum is server-side insert, plus collecting occupied voxels, plus serialisation.

| resolution | octomap | Bonxai | scan period |
|---|---|---|---|
| 0.2 m | ~20 ms | ~3 ms | 100 ms |
| 0.1 m | **~175 ms** (93 + 52 + 30) | ~17 ms (5.6 + 11.5) | 100 ms |

At 0.1 m, octomap_server cannot keep up with a 10 Hz lidar on this CPU and would drop scans. Bonxai uses under a fifth of the budget. Octomap's insert time grows 7× from 0.2 m to 0.1 m, while Bonxai's grows 3.7×.

### Memory

Both figures are the libraries' own accounting. The map stops growing once the area has been seen (around scan 800).

| resolution | octomap | Bonxai |
|---|---|---|
| 0.2 m | 2.0 MB | 1.5 MB (-26 %) |
| 0.1 m | 12.0 MB | 9.1 MB (-24 %) |

### Do the maps agree?

| resolution | occupied voxels (octomap / Bonxai) | IoU | collision checks that differ |
|---|---|---|---|
| 0.2 m | 20,405 / 20,984 | 0.88 | 184 of 200,000 (0.09 %): 73 free only in Bonxai, 111 only in octomap |
| 0.1 m | 65,855 / 95,639 | 0.50 | 216 of 200,000 (0.11 %) |

The voxel-level difference comes from ray casting. octomap traces each ray to the measured point. Bonxai traces it to the centre of the endpoint voxel (`ExactRayIterator`). At the grazing angles a lidar sees along walls and the floor, the two clear different neighbour voxels. At 0.1 m, Bonxai keeps about 45 % more occupied voxels, mostly around surfaces.

For the planner this hardly matters. The robot box is several voxels wide, so the collision answers differ on only about 0.1 % of states, and the disagreements split both ways: neither map is consistently the safer one.

### Collision check speed

| resolution | FCL box vs OcTree (now) | voxel loop, octomap | voxel loop, Bonxai |
|---|---|---|---|
| 0.2 m | 3.75 µs | 12.7 µs | **3.0 µs** |
| 0.1 m | 4.0 µs | 73.5 µs | 20.0 µs |

At 0.2 m, a plain loop over Bonxai (about 7 × 7 × 5 voxels) is already a little faster than FCL. FCL walks the octree hierarchically, so its cost barely changes with resolution, while the loop grows with box volume (8× more voxels at 0.1 m). At finer resolutions Bonxai needs a smarter check to match FCL. Two options, neither measured here:

- **Inflate the obstacles:** dilate the occupied voxels by the box half-extents once per map update. The check then becomes a single lookup. This is cheap to keep current because Bonxai's inserts are fast.
- **Skip empty leaf blocks:** use Bonxai's 8³ leaf masks to skip empty blocks inside the box.

FCL's `distance()` query (`clearance`, used only by the non-default `balanced` objective) has no Bonxai equivalent. It would need a local search or a distance field.

## What switching would involve

1. **Mapping node**: Bonxai already ships `bonxai_ros/bonxai_server`, a component that takes `PointCloud2` and TF and publishes the occupied centres as `PointCloud2`.
   - Its `~/reset` service is `std_srvs/Empty`, like octomap_server's, so the planner's takeoff reset only needs `topics.octomap_reset_service` pointed at it.
   - Its `occupancy_min_z`/`max_z` only filter the published output. Ground points still reach the map, so the 0.3 m input pass-through from `mapping.yaml` would need adding, or the map could move into the planner (next point).
2. **Planner map input**: `/octomap_binary` plus `octomap_msgs` deserialisation would be replaced by either of these:
   - **(a)** Build the `ProbabilisticMap` inside the planner, from the lidar directly or as a composable node sharing the process. This avoids all transport and is the cheapest.
   - **(b)** Subscribe to the occupied-voxel cloud and rebuild a Bonxai grid from it: 21k points, about 330 kB per message at 0.2 m.
3. **`CollisionWorld`**: replace the FCL/OcTree object with the voxel loop (or an inflated grid). `isFree` and `isSegmentFree` keep their interfaces, so OMPL and the follower do not change. The tests would need Bonxai fixtures.
4. **RViz**: [PR #60](https://github.com/facontidavide/Bonxai/pull/60) adds a `BonxaiVoxelMap` message and an RViz2 display. It is open and currently **conflicts** with Bonxai main, whose ROS package was rewritten after the PR was made, so it would need porting. Until then, `bonxai_server`'s `PointCloud2` of occupied centres can be shown with the stock PointCloud2 display (Boxes style, size = resolution).

## Conclusion

- **0.2 m (today):** octomap costs about 20 % of one core at 10 Hz, and FCL checks are fast. Bonxai would cut the mapping cost by about 7× and memory by about a quarter, with the same planning results. That is a real gain but not a pressing one.
- **Finer resolution, denser lidars, or a slower onboard computer:** this is where Bonxai matters. At 0.1 m, octomap_server no longer keeps up with 10 Hz, while Bonxai does with plenty of headroom.
- **Costs of switching:** the map has to stay in the planner's process (Bonxai's serialised form is 31× larger than octomap's), the collision check has to be rewritten without FCL, and the RViz plugin needs porting.

Recommendation: switch, with the Bonxai map in the planner process (option 2a) and an inflated-occupancy collision check, so that finer resolutions stay cheap on the planning side too.

## Reproduce

```bash
# 1. record scans while flying (SITL + mapping + planner running, see the root README)
python3 capture_scans.py tour.bin 130 &
#    ...fly some goals...

# 2. build and run
B=~/ros2_ws/src/Bonxai
g++ -std=c++17 -O2 -march=native -DNDEBUG -I$B/bonxai_core/include -I$B/bonxai_map/include \
  -I/usr/include/eigen3 bench.cpp $B/bonxai_map/src/probabilistic_map.cpp -o bench \
  -lfcl -loctomap -loctomath -lccd
./bench tour.bin 0.2 15 exact     # also: 0.2 15 approx, 0.1 15 exact
```

The raw outputs are `run_exact.txt` (0.2 m, exact rays), `run_approx.txt` (0.2 m, approximate rays) and `run_res01.txt` (0.1 m). The recorded `tour.bin` (61 MB) is not in the repository.

## After switching

The planner now builds the map itself (`OccupancyMap`, `CollisionWorld` in `path_planning`), following option 2a above. Collision checks are the plain voxel loop for now; the inflated grid has not been needed yet at 0.2 m. `bonxai_server` and PR #60 are not used.

These figures were measured afterwards on the same recorded tour and in SITL:

| | octomap + FCL (before) | Bonxai in the node (now) |
|---|---|---|
| Map update per scan, SITL | 13.4 ms insert, plus publish and readBinary | 2.6 ms insert, plus 0.7–5.7 ms snapshot (grows with the map) |
| `isFree` (1.2 × 1.2 × 0.9 m box) | 2.9 µs | 2.3 µs, agrees on 99.97 % of 20k states |
| `clearance` (`balanced` objective) | 458 µs (`fcl::distance`) | 7.8 µs, mean difference 7 mm |
| Walls tour, 4 legs × 3 runs | 10.3–10.6 s open legs, 23–33 s through the walls | the same, 12/12 goals reached |

The switch exposed one problem. On open legs, OMPL's informed sampler returns NaN states once its ellipsoid has collapsed, up to 700k in a 1 s plan. FCL apparently treated those states as free. The voxel loop turned them into an INT_MIN..INT_MAX loop, so they are now rejected before the lookup.

The snapshot is rebuilt from the whole map after every scan, which is why its cost grows with the map: 5.7 ms at 23k occupied voxels. If maps get much larger, rebuild it at a lower rate or only around the voxels that changed.
