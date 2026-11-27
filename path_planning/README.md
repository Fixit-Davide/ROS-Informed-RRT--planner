# path_planning (ROS 2 Lyrical + PX4)

Informed-RRT* collision-avoidance planner for multirotors:
**Bonxai** (occupancy map built in the node from a 3D lidar) → box collision checks →
**OMPL** (Informed-RRT*), commanding **PX4** in OFFBOARD mode over `px4_msgs` (PX4's native
Zenoh client with `rmw_zenoh_cpp`, or the uXRCE-DDS bridge).
Replaces the ROS 1 / MAVROS / ArduPilot version.

## Build

```bash
source ~/ros2_lyrical/install/setup.bash
cd ~/ros2_ws/src && git clone https://github.com/facontidavide/Bonxai.git   # package bonxai_ros
cd ~/ros2_ws && colcon build --packages-up-to path_planning drone_pose
```

Dependencies: `px4_msgs` (must match your PX4 firmware commit, or the topic type hashes
differ and nothing connects), `bonxai_ros` (Bonxai; needs PCL), OMPL, Eigen. Mapping launch:
`drone_pose` (`map -> base_link` tf), `ros_gz_bridge` (SITL only). Visualisation: `rviz2`.

## Run

Every terminal: `source ~/ros2_ws/install/setup.bash` and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd                      # Zenoh router (PX4 and ROS 2 connect to it)
ros2 launch path_planning planner.launch.py            # real drone: pilot switches OFFBOARD + arms
```

Send a goal with RViz2 **2D Goal Pose** (`/goal_pose`). The goal z of RViz is 0, so
`control.default_goal_altitude` is used instead (the ROS 1 version asked for it on stdin).

### Simulation (PX4 SITL + Gazebo)

`sim/models/x500_lidar_3d` is PX4's x500 with a 16-channel 3D `gpu_lidar` (gz topic
`/lidar/points`, frame `lidar_link`). `sim/run_px4_sitl.sh` symlinks it into PX4 and starts
SITL in the `walls` world with PX4's Zenoh middleware:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
path_planning/sim/run_px4_sitl.sh                      # HEADLESS=1 for no Gazebo GUI, PX4_GZ_WORLD=... to change world
ros2 launch path_planning planner.launch.py use_sim_time:=true auto_arm:=true gz_bridge:=true rviz:=true
```

`gz_bridge:=true` bridges the lidar cloud and `/clock` (the cloud is stamped with sim time,
so `use_sim_time:=true` is required). PX4 refuses to arm without a GCS link while
`NAV_DLL_ACT > 0`: run QGroundControl, or set `NAV_DLL_ACT 0` on the ground for headless tests.

### Visualisation

`rviz:=true` opens `config/planner.rviz`: tf, lidar cloud, occupied map voxels,
planned path and waypoints, flown track (`/vehicle_path`) and pose. The map frame is NWU,
so the view is mirrored with respect to Gazebo's ENU world.

## Behaviour

0. Every lidar scan on `topics.cloud` is ray-cast into a Bonxai probabilistic map
   (`mapping.*`: 0.2 m voxels, 15 m max range, points outside z 0.3-6 m dropped), once the tf
   `map -> cloud frame` is known at the scan stamp. The occupied voxels are copied into an
   immutable snapshot for the planning thread, and shown on `~/occupied_voxels`.
1. Streams `OffboardControlMode` + `TrajectorySetpoint` at `control.rate_hz`.
2. Once in OFFBOARD and armed (by the pilot, or by the node if `control.auto_arm`),
   resets the map and takes off to `control.takeoff_altitude` (skipped if already flying).
3. On a goal: plans in a background thread (setpoint stream never stops), B-spline smooths
   and re-interpolates to `planner.waypoint_spacing`. A failed plan is retried up to
   `planner.max_attempts` times, each with twice the solve time.
4. Flies the path without stopping at waypoints (see *Path following*).
5. At `control.replan_check_rate_hz` the rest of the path is checked against the newest
   map. If it is blocked, the drone keeps flying while it replans, braking only if the
   blocked section is within reach; the new path is spliced onto the current reference.
   Paths are planned with a box `robot.planning_margin` larger than the one they are
   checked with, so new voxels right next to a fresh path do not trigger a replan.
6. If the pilot leaves OFFBOARD or disarms, the path is dropped and the node does not
   re-request OFFBOARD until the vehicle has been disarmed.

### Path following

The path is parameterised by arc length with a speed limit profile: `control.max_speed`,
corner speeds from `control.max_lateral_accel` and the turn angle at each waypoint, and a
stop at the goal, all reachable by braking at `control.max_accel`. Each control tick a
reference moves along it with acceleration limited to `max_accel` (braking up to 2x) and
its rate of change to `control.max_jerk`. PX4 receives the reference position, velocity and
acceleration as feed-forward, and a yaw that follows the path tangent at up to
`control.max_yaw_rate`. If the drone falls more than `control.tracking_lag_soft` behind the
reference, the reference slows down, and it waits at `control.tracking_lag_max`.

Before this, every waypoint was a bare position setpoint 1 m ahead: PX4 accelerated towards
it and braked when within `control.acceptance_radius`, so the drone surged and slowed at
every waypoint (speed oscillating 0.6-0.8 m/s on an open 10 m leg). On the same leg it now
accelerates once, cruises at 1.5 m/s and stops once at the goal (16.6 s -> 10.5 s).

## Interfaces

| Topic | Type | Dir |
|-|-|-|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | in |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | in |
| `/lidar/points` | `sensor_msgs/PointCloud2` | in (`topics.cloud`) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | out |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | out |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | out |
| `~/planned_path` | `nav_msgs/Path` | out |
| `~/path_markers` | `visualization_msgs/MarkerArray` | out |
| `~/occupied_voxels` | `visualization_msgs/MarkerArray` | out (`mapping.publish_rate_hz`) |
| `~/reset_map` | `std_srvs/Empty` | service: clear the map |

PX4 main over Zenoh publishes these names without suffix. The uXRCE-DDS bridge may add a
`_vN` suffix to versioned messages: check `ros2 topic list` and set `topics.*` in
`config/planner.yaml` to match.

## Frames

PX4 works in NED. The planner works in `map_frame` (z-up). `frame_convention` selects how
NED is mapped: `nwu` (x, -y, -z, default — same as the `drone_pose` package that publishes
`map -> base_link`) or `enu` (REP-103). It must match the `map -> base_link` tf used to place the lidar scans.

## Changes vs. the ROS 1 version

- MAVROS / ArduPilot GUIDED + `CommandTOL` takeoff → PX4 OFFBOARD via `VehicleCommand` + position setpoints.
- Composable node (`path_planning::InformedRRTPlanner`) + `path_planning_node` executable.
- Interactive `std::cin` prompts (objective, goal altitude) → parameters.
- State space SE(3) → R³ (the drone yaws along the path, so random orientations were wasted samples).
- Octomap + FCL → a Bonxai map inside the node, with voxel-box collision and clearance
  queries (see `../docs/bonxai_eval`).
- Fixed ROS 1 bugs: undefined `ClearanceObjective` (now implemented), objective
  `switch` comparing ints to chars (infinite loop), `dist_err[3]` out-of-bounds, blocking planning
  inside callbacks.
- `balanced` objective uses RRT* (informed sampling is only defined for path length).

## Known issues

- OMPL 1.6's default nearest-neighbour structure (`NearestNeighborsGNAT`) returns duplicate
  motions after `ProblemDefinition::fixInvalidInputStates()` moves a start that lies inside an
  inflated obstacle; `RRTstar::freeMemory()` then frees a motion twice and the node segfaults.
  The planners use `NearestNeighborsSqrtApprox` instead.
- When the straight line to the goal is free, Informed-RRT* finds an almost optimal path at
  once. The informed ellipsoid then collapses (its width is the square root of a value that
  rounds below zero), and the sampler returns NaN states for the rest of the solve time:
  hundreds of thousands per 1 s plan in SITL. NaN passes `satisfiesBounds()`, and a NaN box
  used to turn into an endless voxel loop. NaN states are now rejected before the lookup and
  counted in a warning (`OMPL produced N non-finite states`); the path found is unaffected.
