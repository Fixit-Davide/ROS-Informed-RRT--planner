# Occupied voxels of the 3D occupancy map at one instant.
#
# Voxel (i, j, k) is the cube [i, i+1) x [j, j+1) x [k, k+1) * resolution in header.frame_id
# (Bonxai cell coordinates, floor(p / resolution)). Inside one process the map server
# publishes this as uav_nav_core::VoxelMap through a type adapter: intra-process subscribers
# share the immutable snapshot and this message is only built for other processes.
std_msgs/Header header
float64 resolution
# Cells as x0, y0, z0, x1, y1, z1, ...
int32[] cells
