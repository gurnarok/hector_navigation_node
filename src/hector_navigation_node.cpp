//=================================================================================================
// Copyright (c) 2021, Hannu Henttinen, Satakunta University of Applied Sciences

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//=================================================================================================

#include <ros/ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <hector_path_follower/hector_path_follower.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NavigationPlanner
{
	protected:
		hector_exploration_planner::HectorExplorationPlanner *planner_;
		ros::NodeHandle nh_;
		ros::Publisher plan_pub_;
		ros::Publisher vel_pub_;
		ros::Subscriber goal_sub_;
		geometry_msgs::PoseStamped goalPos;
		costmap_2d::Costmap2DROS *costmap_2d_ros_;
		tf2_ros::Buffer buffer_;
		tf2_ros::TransformListener listener_;
		tf::TransformListener tfl_;
		ros::Timer cmd_vel_gen_timer_;
		ros::Timer plan_gen_timer_;
		pose_follower::HectorPathFollower path_follower_;

	public:
		NavigationPlanner() :listener_(buffer_)
		{
			buffer_.setUsingDedicatedThread(true);
			costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", buffer_);

			path_follower_.initialize(&tfl_);

			planner_ = new hector_exploration_planner::HectorExplorationPlanner();

			planner_->initialize("hector_navigation_node", costmap_2d_ros_);

			goalPos.pose.orientation.w = 1;

			goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NavigationPlanner::goalPosition, this);
			cmd_vel_gen_timer_ = nh_.createTimer(ros::Duration(0.1), &NavigationPlanner::timerCmdVelGeneration, this, false );

			plan_pub_ = nh_.advertise<nav_msgs::Path>("nav_path", 2);
			vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		}

		void goalPosition(const geometry_msgs::PoseStamped &goalPos)
		{
			ROS_INFO("Navigation service called");

			geometry_msgs::PoseStamped robot_pose;
			costmap_2d_ros_->getRobotPose(robot_pose);
			hector_nav_msgs::GetRobotTrajectory::Response plans;

			planner_->makePlan(robot_pose, goalPos, plans.trajectory.poses);

			plans.trajectory.header.frame_id ="map";
			plans.trajectory.header.stamp = ros::Time::now();
			plan_pub_.publish(plans.trajectory);
			ROS_INFO("Setting plan to follower");
			path_follower_.setPlan(plans.trajectory.poses);
		}

		void timerCmdVelGeneration(const ros::TimerEvent& e)
		{
			geometry_msgs::Twist twist;
			path_follower_.computeVelocityCommands(twist);
			vel_pub_.publish(twist);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, ROS_PACKAGE_NAME);

	NavigationPlanner nav;

	ros::spin();

	return 0;
}