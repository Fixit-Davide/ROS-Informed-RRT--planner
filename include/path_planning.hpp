#include "ros/ros.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
//Messages used:
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMavFrame.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/math/transform.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>
#include <std_srvs/Empty.h>



std_srvs::Empty srv;
mavros_msgs::State current_state;
int idx = 0;
ros::Publisher vis_pub;
ros::Publisher traj_pub;
double current_pos[3];
bool replanning;

// construct the state space we are planning in
ompl::base::StateSpacePtr space;

// construct an instance of  space information from this state space
ompl::base::SpaceInformationPtr si;

// create a problem instance
ompl::base::ProblemDefinitionPtr pdef;

// goal state
double prev_goal[7];

// Current pose

double desired_pos[3];
double desired_yaw;
double desired_pitch;
double delta[3];
double dist_err[3];
ompl::geometric::PathGeometric *path_smooth = NULL;

bool replan_flag = false;

std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

std::shared_ptr<fcl::CollisionGeometry> tree_obj;

// Flag for initialization
bool set_start = false;
