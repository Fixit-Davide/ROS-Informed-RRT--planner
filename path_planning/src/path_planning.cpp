#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <message_filters/subscriber.h>
#include "visualization_msgs/Marker.h"

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMavFrame.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>

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
// clear octomap service
#include <std_srvs/Empty.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std_srvs::Empty srv;

// Declear some global variables
mavros_msgs::State current_state;
// ROS publishers
int idx = 0;
ros::Publisher vis_pub;
ros::Publisher traj_pub;
double current_pos[3];
bool replanning;
class planner
{
public:
	// Callback functions for position of the drone getting data from the Fake_GPS plugin.
	void checkPos(double x, double y, double z)
	{
		current_pos[0] = x;
		current_pos[1] = y;
		current_pos[2] = z;
		// std::cout << " " << current_pos[0] << " " << current_pos[1] << " " << current_pos[2] << std::endl;
	}
	// Initialise the alg.
	void init_start(void)
	{

		if (!set_start)
		{
			std::cout << "PLANNER STARTED!!!" << std::endl;
			set_start = true;
		}
	}

	// Setting the starting point using the Drone Odometry.
	void setStart(double x, double y, double z)
	{
		ob::ScopedState<ob::SE3StateSpace> start(space);
		start->setXYZ(x, y, z);
		start->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(prev_goal[3], prev_goal[4], prev_goal[5], prev_goal[6]);
		pdef->clearStartStates();
		pdef->addStartState(start);
	}

	// Setting the Goal relatives coordinates for the mission.
	void setGoal(double x, double y, double z, double ax, double ay, double az, double a)
	{

		if (prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z)
		{
			std::cout << "Enter the Altitude (m) of the Goal Point: ";
			std::cin >> z;
			ob::ScopedState<ob::SE3StateSpace> goal(space);
			goal->setXYZ(x, y, z);
			prev_goal[0] = x;
			prev_goal[1] = y;
			prev_goal[2] = z;
			prev_goal[3] = ax;
			prev_goal[4] = ay;
			prev_goal[5] = az;
			prev_goal[6] = a;

			goal->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(ax, ay, az, a);
			pdef->clearGoal();
			pdef->setGoalState(goal);
			std::cout << "Goal_Point set to: " << x << " " << y << " " << z << std::endl;
			// std::cout << "Final Attitude set to: " << ax << " " << ay << " " << az << std::endl;
			if (set_start)
				plan();
		}
	}
	// Update the Octomap
	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
	{
		tree_obj = map;
	}

	// Constructor
	planner(void)
	{
		// Change the values to modify the dimesion of the drone in the Space. X-Y-Z (meters)
		Quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.55, 0.55, 0.50));
		// Set the resolution of the octomap, this parameter can be left unchanged.
		fcl::OcTree *tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
		tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
		space = ob::StateSpacePtr(new ob::SE3StateSpace());

		// create a start state
		ob::ScopedState<ob::SE3StateSpace> start(space);

		// create a goal state
		ob::ScopedState<ob::SE3StateSpace> goal(space);

		// set the bounds for the R^3 part of SE(3)
		ob::RealVectorBounds bounds(3);

		bounds.setLow(0, -20);
		bounds.setHigh(0, 20);
		bounds.setLow(1, -20);
		bounds.setHigh(1, 20);
		bounds.setLow(2, 0.5);
		bounds.setHigh(2, 4.0);
		space->as<ob::SE3StateSpace>()->setBounds(bounds);

		// construct an instance of  space information from this state space
		si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

		start->setXYZ(0, 0, 0);

		start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// start.random();

		goal->setXYZ(0.0, 0.0, 0.0);
		prev_goal[0] = 0;
		prev_goal[1] = 0;
		prev_goal[2] = 0;
		prev_goal[3] = 0;
		prev_goal[4] = 0;
		prev_goal[5] = 0;
		prev_goal[6] = 1;

		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		// goal.random();

		// set state validity checking for this space
		si->setStateValidityChecker(std::bind(&planner::isStateValid, this, std::placeholders::_1));

		// create a problem instance
		pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);

		// fix invalid Start/Goal States
		// If the starting point or the goal point are in Invalid position, the alg. will search the nearest position within a delta.
		// In this case the range is setted as 1.2meters.
		pdef->fixInvalidInputStates(1.2, 1.2, 10);

		pdef->getSolutionDifference();
		bool obj_var = false;
		while (obj_var == false)
		{
			int opt_var;
			std::cout << "Set the Optimization Objective:\n 1: PathLength with Cost to Go. \n 2: Balanced Objective. \n 3: Treshold.";
			std::cin >> opt_var;
			switch (opt_var)
			{
			case '1':
				pdef->setOptimizationObjective(planner::getPathLengthObjWithCostToGo(si));
				break;
			case '2':
				pdef->setOptimizationObjective(planner::getBalancedObjective(si));
				break;
			case '3':
				pdef->setOptimizationObjective(planner::getThresholdPathLengthObj(si));
			default:
				break;
			}
		}
		// set Optimization objective
		// std::cout << "OMPL started " << std::endl;
	}
	// Destructor
	~planner()
	{
	}

	void replan(void)
	{
		if (path_smooth != NULL && set_start)
		{
			// std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
			if (path_smooth->getStateCount() > 2)
			{
				for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
				{
					if (!replan_flag)
						replan_flag = !isStateValid(path_smooth->getState(idx));
					else
						break;
				}
				if (replan_flag)
				{
					// idx = 0;
					replan_flag = false;
					// path_smooth->clear();
					pdef->clearSolutionPaths();
					std::cout << "REPLAN!!!" << std::endl;
					plan();
				}

				else
				{
					// std::cout << "Replanning is not required" << std::endl;
				}
			}
		}
	}

	void plan(void)
	{

		og::InformedRRTstar *informedrrtstar = new ompl::geometric::InformedRRTstar(si);
		informedrrtstar->setRange(4);
		// informedrrtstar->setGoalBias(planner_goal_bias_);
		informedrrtstar->setKNearest(false);

		ob::PlannerPtr plan(informedrrtstar);
		// create a planner for the defined space
		// ob::PlannerPtr plan(new og::InformedRRTstar(si));

		// set the problem we are trying to solve for the planner
		plan->setProblemDefinition(pdef);

		// perform setup steps for the planner
		plan->setup();

		// print the settings for this space
		si->printSettings(std::cout);

		// print the problem settings
		pdef->print(std::cout);

		// attempt to solve the problem within 1 seconds of planning time
		ob::PlannerStatus solved = plan->solve(2);

		if (solved)
		{
			std::cout << "Found a solution:" << std::endl;
			ob::PathPtr path = pdef->getSolutionPath();
			og::PathGeometric *pth = pdef->getSolutionPath()->as<og::PathGeometric>();
			pth->printAsMatrix(std::cout);

			// print the path to screen
			path->print(std::cout);

			og::PathSimplifier *pathBSpline = new og::PathSimplifier(si);
			path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
			pathBSpline->smoothBSpline(*path_smooth, 2);
			// pathBSpline->simplifyMax(*path_smooth);

			// Publish path as markers
			visualization_msgs::Marker marker;
			marker.action = visualization_msgs::Marker::DELETEALL;
			vis_pub.publish(marker);

			for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
			{
				ros::Rate rate3(40);
				// cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

				// extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

				// extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				marker.header.frame_id = "map";
				marker.header.stamp = ros::Time();
				marker.ns = "path";
				marker.id = idx;
				marker.type = visualization_msgs::Marker::SPHERE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = pos->values[0];
				marker.pose.position.y = pos->values[1];
				marker.pose.position.z = pos->values[2];
				marker.pose.orientation.x = rot->x;
				marker.pose.orientation.y = rot->y;
				marker.pose.orientation.z = rot->z;
				marker.pose.orientation.w = rot->w;
				marker.scale.x = 0.8;
				marker.scale.y = 0.8;
				marker.scale.z = 0.5;
				marker.color.a = 0.5;
				marker.color.r = 0;
				marker.color.g = 0.3;
				marker.color.b = 0;
				vis_pub.publish(marker);

				std::cout << "Published marker: " << idx << std::endl;
				rate3.sleep();
			}

			geometry_msgs::PoseStamped msg;
			dist_err[0] = 0;
			dist_err[1] = 0;
			dist_err[2] = 0;

			idx = 0;
			while (idx < path_smooth->getStateCount())
			{
				ros::Rate rate2(50);

				// cast the abstract state type to the type we expect
				const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();

				// extract the first component of the state and cast it to what we expect
				const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

				// extract the second component of the state and cast it to what we expect
				const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "base_link";
				// Position MESSAGE
				msg.pose.position.x = pos->values[0];
				msg.pose.position.y = pos->values[1];
				msg.pose.position.z = pos->values[2];
				// stores values in vector
				desired_pos[0] = pos->values[0];
				desired_pos[1] = pos->values[1];
				desired_pos[2] = pos->values[2];

				// delta
				delta[0] = current_pos[0] - desired_pos[0]; // x
				delta[1] = current_pos[1] - desired_pos[1]; // y
				delta[2] = current_pos[2] - desired_pos[2]; // z
				// YAW control
				desired_yaw = atan2(delta[1], delta[0]) - M_PI;
				desired_pitch = -atan2(delta[2], sqrt(delta[0] * delta[0] + delta[1] * delta[1]));
				// generate the quaternion:
				tf2::Quaternion quat;
				quat.setRPY(0, 0, desired_yaw);
				quat.normalize();
				// Orientation MESSAGE
				msg.pose.orientation.x = quat[0];
				msg.pose.orientation.y = quat[1];
				msg.pose.orientation.z = quat[2];
				msg.pose.orientation.w = quat[3];

				// Give values:
				// std::cout << "Published waypoint n. : " << idx << std::endl;
				// std::cout << "Waypoint: " << msg.pose.position.x << " " << msg.pose.position.y << " " << msg.pose.position.z << std::endl;
				// std::cout << "Attitude: " << msg.pose.orientation.x << " " << msg.pose.orientation.y << " " << msg.pose.orientation.z << std::endl;
				// Wait to reach the sent point!!!
				dist_err[0] = desired_pos[0] - current_pos[0]; // x
				dist_err[1] = desired_pos[1] - current_pos[1]; // y
				dist_err[2] = desired_pos[2] - current_pos[2]; // z
				ros::Rate r(2);								   // 50 Hz!
				// Compute the euclidean distance and loop:
				while (sqrt(pow(dist_err[0], 2) + pow(dist_err[1], 2) + pow(dist_err[3], 2)) > 0.2)
				{
					traj_pub.publish(msg);
					//	std::cout << "dist:" << dist_err[0] << " " << dist_err[1] << " " << dist_err[2] << std::endl;
					// std::cout << "Waiting to reach point:" << idx << std::endl;
					dist_err[0] = desired_pos[0] - current_pos[0]; // x
					dist_err[1] = desired_pos[1] - current_pos[1]; // y
					dist_err[2] = desired_pos[2] - current_pos[2]; // z
					ros::spinOnce();
					r.sleep();
				}
				idx++;
				replan();

				rate2.sleep();
			}

			// Clear memory
			pdef->clearSolutionPaths();
			replan_flag = false;
			std::cout << "GOAL REACHED!!!" << std::endl;
		}
		else
			std::cout << "No Solution Found!" << std::endl;
	}

private:
	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	// goal state
	double prev_goal[7];

	// Current pose

	double desired_pos[3];
	double desired_yaw;
	double desired_pitch;
	double delta[3];
	double dist_err[3];
	og::PathGeometric *path_smooth = NULL;

	bool replan_flag = false;

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry> tree_obj;

	// Flag for initialization
	bool set_start = false;

	bool isStateValid(const ob::State *state)
	{
		// cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

		// extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		// extract the second component of the state and cast it to what we expect
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		fcl::CollisionObject treeObj((tree_obj));
		fcl::CollisionObject aircraftObject(Quadcopter);

		// check validity of state defined by pos & rot
		fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
		fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		aircraftObject.setTransform(rotation, translation);
		fcl::CollisionRequest requestType(1, false, 1, false);
		fcl::CollisionResult collisionResult;
		fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

		return (!collisionResult.isCollision());
	}

	// Returns a structure representing the optimization objective to use
	// for optimal motion planning. This method returns an objective which
	// attempts to minimize the length in configuration space of computed
	// paths.
	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr &si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostThreshold(ob::Cost(4));
		return obj;
	}

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si)
	{
		ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
		obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
		return obj;
	}

	ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr &si)
	{
		ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
		ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

		ob::MultiOptimizationObjective *obj = new ob::MultiOptimizationObjective(si);
		obj->addObjective(lengthObj, 10.0);
		obj->addObjective(clearObj, 1.0);

		return obj;
	}
};

void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner *planner_ptr)
{
	ros::Rate rate(0.2);
	// ros::service::call("/octomap_server/reset", srv);
	//  convert octree to collision object
	octomap::OcTree *tree_oct = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
	fcl::OcTree *tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	// Update the octree used for collision checking
	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
	// planner_ptr->replan(); // decommentare
	rate.sleep();
}

void posCb(const geometry_msgs::PoseStamped::ConstPtr &pos, planner *planner_ptr)
{

	planner_ptr->checkPos(pos->pose.position.x, pos->pose.position.y, pos->pose.position.z);
}
/*
void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner *planner_ptr)
{
	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	planner_ptr->init_start();
}
*/
void odomCb(const geometry_msgs::PoseStamped::ConstPtr &msg, planner *planner_ptr)
{
	planner_ptr->setStart(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	planner_ptr->init_start();
}
void goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg, planner *planner_ptr)
{
	planner_ptr->setGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
						 msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

void state_callback(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}
/*
void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
{
	planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_planner");
	ros::NodeHandle n;

	planner planner_object;
	ros::MultiThreadedSpinner spinner(4);
	ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(&octomapCallback, _1, &planner_object));		  //
	ros::Subscriber odom_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&odomCb, _1, &planner_object)); //
	ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&goalCb, _1, &planner_object));
	vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	traj_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

	// mavros/fake_gps/mocap/tf  ---> position of the drone using the ultra wide band coordinates.
	ros::Subscriber position = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&posCb, _1, &planner_object));
	ros::Duration(0.5).sleep();

	if (srv_setMode.)
	{
	}
	////////////////////////////////////////////
	/////////////////GUIDED/////////////////////
	////////////////////////////////////////////
	ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	mavros_msgs::SetMode srv_setMode;
	srv_setMode.request.base_mode = 0;
	srv_setMode.request.custom_mode = "GUIDED";
	if (cl.call(srv_setMode))
	{
		ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
	}
	else
	{
		ROS_ERROR("Failed SetMode");
		return -1;
	}
	ros::Duration(2).sleep();

	////////////////////////////////////////////
	///////////////////ARM//////////////////////
	////////////////////////////////////////////
	ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	mavros_msgs::CommandBool srv;
	srv.request.value = true;
	if (arming_cl.call(srv))
	{
		ROS_ERROR("Arming drone ok value %d:", srv.response.success);
	}
	else
	{
		ROS_ERROR("Failed arming or disarming");
	}
	ros::Duration(5).sleep();

	///////RESET THE INITIAL MAP GENERATED/////////
	ROS_INFO("Resetting the initial Octomap!");
	ros::service::call("/octomap_server/reset", srv);

	////////////////////////////////////////////
	/////////////////TAKEOFF////////////////////
	////////////////////////////////////////////
	ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = 2.0;
	srv_takeoff.request.latitude = 0;
	srv_takeoff.request.longitude = 0;
	srv_takeoff.request.min_pitch = 0;
	srv_takeoff.request.yaw = 0;
	if (takeoff_cl.call(srv_takeoff))
	{
		ROS_INFO("Takeoff 2m sent.");
	}
	else
	{
		ROS_ERROR("Failed Takeoff");
	}

	spinner.spin();
}
