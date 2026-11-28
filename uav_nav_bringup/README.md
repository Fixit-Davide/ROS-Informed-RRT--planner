# uav_nav_bringup

Launch files, parameters and the RViz view of the UAV navigation servers, plus the PX4 SITL
model and worlds. The [top-level README](../README.md) explains how the system works; this
page is the short reference.

## Build

```bash
source ~/ros2_lyrical/install/setup.bash
cd ~/ros2_ws/src && git clone https://github.com/facontidavide/Bonxai.git   # package bonxai_ros
cd ~/ros2_ws && colcon build --packages-up-to uav_nav_bringup drone_pose
```

Dependencies: `px4_msgs` (must match your PX4 firmware commit, or the topic type hashes
differ and nothing connects), `bonxai_ros` (Bonxai; needs PCL), OMPL, Eigen, `nav2_msgs` and
`nav2_lifecycle_manager`. Mapping launch: `drone_pose` (`map -> base_link` tf),
`ros_gz_bridge` (SITL only). Visualisation: `rviz2`.

Unit tests:

```bash
colcon build --packages-up-to uav_nav_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select uav_nav_core uav_planners uav_controller_server
colcon test-result --verbose
```

## Run

Every terminal: `source ~/ros2_ws/install/setup.bash` and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd                         # Zenoh router (PX4 and ROS 2 connect to it)
ros2 launch uav_nav_bringup bringup.launch.py             # real drone: pilot switches OFFBOARD + arms
```

Simulation (PX4 SITL + Gazebo, `x500_lidar_3d` = PX4's x500 with a 16-channel 3D lidar):

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
uav_nav_bringup/sim/run_px4_sitl.sh                       # walls world; PX4_GZ_WORLD=obstacle_course, HEADLESS=1, -d under nohup
ros2 launch uav_nav_bringup bringup.launch.py use_sim_time:=true auto_arm:=true gz_bridge:=true rviz:=true
```

PX4 refuses to arm without a GCS link while `NAV_DLL_ACT > 0`: run QGroundControl, or set
`NAV_DLL_ACT 0` on the ground for headless tests (and back before flying a real vehicle).

### Launch arguments

| Argument | Default | |
|-|-|-|
| `params_file` | `config/nav.yaml` | parameters of every server |
| `use_composition` | `true` | one component container with intra-process comms (zero-copy map); `false`: one process per server |
| `autostart` | `true` | the lifecycle manager configures and activates the servers |
| `use_sim_time` | `false` | `true` in SITL (the lidar cloud is stamped with sim time) |
| `auto_arm` | `false` | the controller switches PX4 to OFFBOARD and arms (SITL) |
| `mapping` | `true` | also start `drone_pose` and the `base_link -> lidar_link` tf |
| `gz_bridge` | `false` | bridge the Gazebo lidar cloud and `/clock` |
| `rviz` | `false` | open `config/nav.rviz` |
| `log_level` | `info` | |

### Goals

RViz2 **2D Goal Pose** (`/goal_pose`), or the Nav2 action:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: -8.0, z: 2.0}}}}" --feedback
```

A goal below `bounds.min` z (RViz sends z = 0) is flown at `navigator.default_goal_altitude`;
goals outside `bounds` are clamped into them. Cancelling the action stops the drone along
its path and hovers.

## Nodes and interfaces

| Node | Package | In | Out |
|-|-|-|-|
| `map_server` | `uav_map_server` | `/lidar/points`, tf | `voxel_map` (`uav_nav_core::VoxelMap`), `~/occupied_voxels`, `~/reset_map` |
| `planner_server` | `uav_planner_server` + `uav_planners` plugins | `voxel_map` | `compute_path` action, `~/global_path`, `~/local_path`, `~/path_markers` |
| `controller_server` | `uav_controller_server` | `voxel_map`, `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status` | `/fmu/in/*` setpoints and commands, `follow_path` action, `~/get_replan_start`, `~/state`, `~/trajectory` |
| `navigator` | `uav_navigator` | `/goal_pose`, `/controller_server/state` | `navigate_to_pose` action (`nav2_msgs`) |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | | brings the four servers up in that order |

`voxel_map` is volatile, keep-last-1 on purpose: with transient-local durability rclcpp
converts every message to keep it for late joiners, and the zero-copy path is lost. The map
server republishes every `republish_period` (1 s) when no scan arrives.

PX4 main over Zenoh publishes the `/fmu` names without suffix. The uXRCE-DDS bridge may add a
`_vN` suffix to versioned messages: check `ros2 topic list` and set the controller's
`topics.*` to match.

## Frames

PX4 works in NED. The servers work in `map_frame` (z-up). The controller's
`frame_convention` selects how NED is mapped: `nwu` (default, same as the `drone_pose`
package that publishes `map -> base_link`) or `enu` (REP-103). It must match the
`map -> base_link` tf used to place the lidar scans.
