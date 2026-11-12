# path_planning (ROS 2 Lyrical + PX4)

Informed-RRT* collision-avoidance planner for multirotors:
**Octomap** (map) → **FCL** (collision checks of a box-shaped drone) → **OMPL** (Informed-RRT*),
commanding **PX4** in OFFBOARD mode through the uXRCE-DDS bridge (`px4_msgs`).
Replaces the ROS 1 / MAVROS / ArduPilot version.

## Build

```bash
source ~/ros2_lyrical/install/setup.bash
cd ~/ros2_ws && colcon build --packages-up-to path_planning octomap_server
```

Dependencies: `px4_msgs` (must match your PX4 firmware branch), `octomap_msgs`,
`octomap_server` (mapping launch only), OMPL, `libfcl-dev` (with Octomap support), `liboctomap-dev`.

## Run

```bash
MicroXRCEAgent udp4 -p 8888              # PX4 <-> ROS 2 bridge
ros2 launch path_planning planner.launch.py             # real drone: pilot switches OFFBOARD + arms
ros2 launch path_planning planner.launch.py auto_arm:=true use_sim_time:=true   # SITL
```

Send a goal with RViz2 **2D Goal Pose** (`/goal_pose`). The goal z of RViz is 0, so
`control.default_goal_altitude` is used instead (the ROS 1 version asked for it on stdin).

## Behaviour

1. Streams `OffboardControlMode` + `TrajectorySetpoint` at `control.rate_hz`.
2. Once in OFFBOARD and armed (by the pilot, or by the node if `control.auto_arm`),
   resets the octomap and takes off to `control.takeoff_altitude` (skipped if already flying).
3. On a goal: plans in a background thread (setpoint stream never stops), B-spline smooths,
   re-interpolates to `planner.waypoint_spacing`, and flies the waypoints, yawing along the path.
4. At `control.replan_check_rate_hz` the remaining path is checked against the newest
   octomap; if it is blocked the drone holds position and replans.
5. If the pilot leaves OFFBOARD or disarms, the path is dropped and the node does not
   re-request OFFBOARD until the vehicle has been disarmed.

## Interfaces

| Topic | Type | Dir |
|-|-|-|
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | in |
| `/fmu/out/vehicle_status_v5` | `px4_msgs/VehicleStatus` | in |
| `/octomap_binary` | `octomap_msgs/Octomap` | in |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | out |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | out |
| `/fmu/in/vehicle_command_v1` | `px4_msgs/VehicleCommand` | out |
| `~/planned_path` | `nav_msgs/Path` | out |
| `~/path_markers` | `visualization_msgs/MarkerArray` | out |

PX4 versioned messages get a `_vN` topic suffix; check `ros2 topic list` and set
`topics.*` in `config/planner.yaml` to match your firmware.

## Frames

PX4 works in NED. The planner works in `map_frame` (z-up). `frame_convention` selects how
NED is mapped: `nwu` (x, -y, -z, default — same as the `drone_pose` package that publishes
`map -> base_link`) or `enu` (REP-103). It must match the tf used to build the octomap.

## Changes vs. the ROS 1 version

- MAVROS / ArduPilot GUIDED + `CommandTOL` takeoff → PX4 OFFBOARD via `VehicleCommand` + position setpoints.
- Composable node (`path_planning::InformedRRTPlanner`) + `path_planning_node` executable.
- Interactive `std::cin` prompts (objective, goal altitude) → parameters.
- State space SE(3) → R³ (the drone yaws along the path, so random orientations were wasted samples).
- FCL 0.5 API → FCL 0.7 (`fcl::OcTreed`, `fcl::Boxd`, …).
- Fixed ROS 1 bugs: undefined `ClearanceObjective` (now implemented with FCL distance), objective
  `switch` comparing ints to chars (infinite loop), `dist_err[3]` out-of-bounds, blocking planning
  inside callbacks.
- `balanced` objective uses RRT* (informed sampling is only defined for path length).
