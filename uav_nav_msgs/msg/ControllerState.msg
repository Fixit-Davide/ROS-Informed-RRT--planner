# State of the controller server (the node that streams PX4 offboard setpoints).
uint8 WAIT_FOR_ODOM=0    # no vehicle odometry yet
uint8 PRE_OFFBOARD=1     # streaming hold setpoints, waiting for OFFBOARD + armed
uint8 TAKEOFF=2
uint8 HOVER=3            # holding hold_position, ready for a path
uint8 FOLLOW_PATH=4

std_msgs/Header header
uint8 state
geometry_msgs/Point position        # vehicle position in header.frame_id
geometry_msgs/Point hold_position   # where it holds (HOVER) / the reference (FOLLOW_PATH)
