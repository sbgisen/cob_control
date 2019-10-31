/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Matthias Gruhler, email:Matthias.Gruhler@ipa.fhg.de
 *
 * Date of creation: February 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <cob_collision_velocity_filter.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <log4cxx/logger.h>

// Constructor
CollisionVelocityFilter::CollisionVelocityFilter() {
	// create node handle
	nh_ = ros::NodeHandle("~");
	initTfListener();
	m_mutex = PTHREAD_MUTEX_INITIALIZER;

	// node handle to get footprint from parameter server
	std::string costmap_parameter_source;
	if (!nh_.hasParam("costmap_parameter_source"))
		ROS_WARN(
				"Checking default source [/local_costmap_node/costmap] for costmap parameters");
	nh_.param("costmap_parameter_source", costmap_parameter_source,
			std::string("/local_costmap_node/costmap"));

	ros::NodeHandle local_costmap_nh_(costmap_parameter_source);

	nh_.param("costmap_obstacle_treshold", costmap_obstacle_treshold_, 50);

	// implementation of topics to publish (command for base and list of relevant obstacles)
	topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
	topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::OccupancyGrid>(
			"relevant_obstacles_grid", 1);
	topic_pub_lookat_ = nh_.advertise<geometry_msgs::PoseStamped>(
			"collision_lookat", 1);
	// subscribe to twist-movement of teleop
	joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist",
			10,
			boost::bind(&CollisionVelocityFilter::joystickVelocityCB, this,
					_1));
	// subscribe to the costmap to receive inflated cells
	obstacles_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("obstacles", 10,
			boost::bind(&CollisionVelocityFilter::obstaclesCB, this, _1));

	// read parameters from parameter server
	// parameters from costmap
	if (!local_costmap_nh_.hasParam(costmap_parameter_source + "/global_frame"))
		ROS_WARN("Used default parameter for global_frame [/map]");
	local_costmap_nh_.param(costmap_parameter_source + "/global_frame",
			global_frame_, std::string("/map"));

	if (!local_costmap_nh_.hasParam(
			costmap_parameter_source + "/robot_base_frame"))
		ROS_WARN("Used default parameter for robot_frame [/base_link]");
	local_costmap_nh_.param(costmap_parameter_source + "/robot_base_frame",
			robot_frame_, std::string("/base_link"));

	if (!nh_.hasParam("slowdown_radious"))
		ROS_WARN("Used default parameter for slowdown_radious [1.5]");
	nh_.param("slowdown_radious", slowdown_radious_, 1.5);\
	if (slowdown_radious_ < 0.2) {
		slowdown_radious_ = 0.2; //TODO: parametarize
	}
	if (!nh_.hasParam("stop_radious"))
		ROS_WARN("Used default parameter for stop_radious [1.0]");
	nh_.param("stop_radious", stop_radious_, 1.0);
	if (stop_radious_ > slowdown_radious_ - 0.1) {
		stop_radious_ = slowdown_radious_ - 0.1; //TODO: parametarize
	}
	if (!nh_.hasParam("collision_radious"))
		ROS_WARN("Used default parameter for collision_radious [0.5]");
	nh_.param("collision_radious", collision_radious_, 0.5);
	if (collision_radious_ < 0.1) {
		collision_radious_ = 0.1; //TODO: parametarize
	}
	if (!nh_.hasParam("vx_max"))
		ROS_WARN("Used default parameter for vx_max [1.0]");
	nh_.param("vx_max", vx_max_, 1.0);
	braking_distance_ = slowdown_radious_ - stop_radious_;
	desired_stopping_time_ = (2.0 * braking_distance_) / vx_max_;
	breaking_accel_ = vx_max_ / desired_stopping_time_;
	stopping_interval_ = desired_stopping_time_;

	last_time_ = ros::Time::now();
	vx_last_ = 0.0;
}

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter() {
}

void CollisionVelocityFilter::initTfListener() {
	double cache_time_ = 3.0;
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(cache_time_), true));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(cache_time_);
}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(
		const geometry_msgs::Twist::ConstPtr &twist) {
	pthread_mutex_lock(&m_mutex);
	robot_twist_linear_ = twist->linear;
	robot_twist_angular_ = twist->angular;
	pthread_mutex_unlock(&m_mutex);

	//get robot direction

	if (tfBuffer_ptr->canTransform(global_frame_, robot_frame_, ros::Time(0))) {
		tf_robot_ = tfBuffer_ptr->lookupTransform(global_frame_, robot_frame_,
				ros::Time(0));
		tf2::Quaternion robot_quat(tf_robot_.transform.rotation.x,
				tf_robot_.transform.rotation.y, tf_robot_.transform.rotation.z,
				tf_robot_.transform.rotation.w);
		geometry_msgs::Vector3 robot_euler;
		tf2::Matrix3x3(robot_quat).getEulerYPR(robot_euler.z, robot_euler.y,
				robot_euler.x);
		robot_angle_ = robot_euler.z;
		robot_quat.setRPY(0.0, 0.0, robot_euler.z);
		if (fabs(robot_twist_linear_.x) < 0.001) {
			collision_look_angle_ = robot_angle_;
		} else if (robot_twist_linear_.x > 0.0) {
			double angle_diff = stopping_interval_ * robot_twist_angular_.z;
			if(angle_diff > M_PI / 2.0){
				angle_diff = M_PI / 2.0;
			}else if(angle_diff < -M_PI / 2.0){
				angle_diff = -M_PI / 2.0;
			}
			collision_look_angle_ = robot_angle_ + angle_diff;
		} else {
			double angle_diff = stopping_interval_ * robot_twist_angular_.z;
			if(angle_diff > M_PI / 2.0){
				angle_diff = M_PI / 2.0;
			}else if(angle_diff < -M_PI / 2.0){
				angle_diff = -M_PI / 2.0;
			}
			collision_look_angle_ = - M_PI + robot_angle_ + angle_diff;
		}
		tf2::Quaternion lookat_quat;
		lookat_quat.setRPY(0.0, 0.0, collision_look_angle_);
		//publish debug topic
		geometry_msgs::PoseStamped lookat_tmp;
		lookat_tmp.header.stamp = ros::Time::now();
		lookat_tmp.header.frame_id = global_frame_;
		lookat_tmp.pose.position.x = tf_robot_.transform.translation.x;
		lookat_tmp.pose.position.y = tf_robot_.transform.translation.y;
		lookat_tmp.pose.position.z = tf_robot_.transform.translation.z;
		lookat_tmp.pose.orientation.x = lookat_quat.getX();
		lookat_tmp.pose.orientation.y = lookat_quat.getY();
		lookat_tmp.pose.orientation.z = lookat_quat.getZ();
		lookat_tmp.pose.orientation.w = lookat_quat.getW();
		topic_pub_lookat_.publish(lookat_tmp);
	} else {
		ROS_INFO("Can't find origin TF [%s]", robot_frame_.c_str());
	}
	// check for relevant obstacles
	obstacleHandler();
	// stop if we are about to run in an obstacle
	performControllerStep();

}

// obstaclesCB reads obstacles from costmap
void CollisionVelocityFilter::obstaclesCB(
		const nav_msgs::OccupancyGrid::ConstPtr &obstacles) {
	pthread_mutex_lock(&m_mutex);
	if (obstacles->data.size() != 0)
		costmap_received_ = true;
	last_costmap_received_ = *obstacles;
	pthread_mutex_unlock(&m_mutex);
}

// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep() {

	geometry_msgs::Twist cmd_vel, cmd_vel_in;

	cmd_vel_in.linear = robot_twist_linear_;
	cmd_vel_in.angular = robot_twist_angular_;

	cmd_vel.linear = robot_twist_linear_;
	cmd_vel.angular = robot_twist_angular_;

	ros::Duration dt_ros = ros::Time::now() - last_time_;
	last_time_ = ros::Time::now();
	double dt = dt_ros.toSec();
	//limit max velocity
	if(cmd_vel.linear.x > vx_max_){
		cmd_vel.linear.x = vx_max_;
	}else if(cmd_vel.linear.x < -vx_max_){
		cmd_vel.linear.x = -vx_max_;
	}

	//check collision
	if(vx_last_ * cmd_vel.linear.x < 0){
		vx_last_ = 0; //reset vx_last_ when liner-x plus/minus has changed.
	}
	if(nearest_obstacle_linear_ < slowdown_radious_ && nearest_obstacle_direction_ * cmd_vel.linear.x > 0.000001){
		double breaking_length = nearest_obstacle_linear_ - stop_radious_;

		if(breaking_length < 0.000001){
			ROS_WARN("Found obstacle CLOSE!: ObstacleDistance[%f], StopDistance[%f] ", nearest_obstacle_linear_, stop_radious_);
			breaking_length = 0;
			cmd_vel.linear.x = 0;
			costmap_received_ = true;
		 }else{
			double deaccel = (vx_last_ * vx_last_) / (2.0 * breaking_length);
			double vel_deaccel = vx_last_ - nearest_obstacle_direction_ * deaccel * dt;
			double vel_desired_stopping = nearest_obstacle_direction_ * sqrt(2.0 * breaking_accel_ * breaking_length );
			if(nearest_obstacle_direction_ > 0 && vel_desired_stopping > vel_deaccel){
				vel_deaccel = vel_desired_stopping;
			}else if(nearest_obstacle_direction_ < 0 && vel_desired_stopping < vel_deaccel){
				vel_deaccel = vel_desired_stopping;
			}
			if(vel_deaccel * vx_last_ < -0.001 || vel_deaccel * cmd_vel.linear.x < -0.001){
				vel_deaccel = 0;
				ROS_WARN("Deaccel = 0 (direction changed)");
			}
			if(fabs(vel_deaccel) < fabs(cmd_vel.linear.x)){
				 cmd_vel.linear.x = vel_deaccel;
			}
			ROS_DIBUG("Found an obstacle: Distance[%f], breaking:[%f], stop_radious:[%f], vel:[%f], vdes:[%f]", nearest_obstacle_linear_, breaking_length, stop_radious_, vel_deaccel,vel_desired_stopping);
			if(nearest_obstacle_direction_ * cmd_vel.linear.x < 0){
				nearest_obstacle_linear_ += fabs(cmd_vel.linear.x) * dt;
			}else{
				nearest_obstacle_linear_ -= fabs(cmd_vel.linear.x) * dt;
			}
		 }
	 }
	vx_last_ = cmd_vel.linear.x;
	topic_pub_command_.publish(cmd_vel);
	return;
}

void CollisionVelocityFilter::obstacleHandler() {
	if (!costmap_received_) {
//		ROS_DEBUG("No costmap has been received by cob_collision_velocity_filter, the robot will drive without obstacle avoidance!");
//		return;
	}
	double cur_distance_to_center, cur_distance_to_border;
	double obstacle_theta_robot, obstacle_delta_theta_robot,
			obstacle_dist_vel_dir;
	bool cur_obstacle_relevant;
	geometry_msgs::Point cur_obstacle_robot;
	geometry_msgs::Point zero_position;
	zero_position.x = 0.0f;
	zero_position.y = 0.0f;
	zero_position.z = 0.0f;

	//find relevant obstacles
//	pthread_mutex_lock(&m_mutex);
	relevant_obstacles_.header = last_costmap_received_.header;
	relevant_obstacles_.info = last_costmap_received_.info;
	relevant_obstacles_.data.clear();

	double nearest_obstacle_distance_tmp = slowdown_radious_;
	nearest_obstacle_direction_ = 0;

	for (unsigned int i = 0; i < last_costmap_received_.data.size(); i++) {
		if (last_costmap_received_.data[i] == -1) {
			relevant_obstacles_.data.push_back(-1);
		} else if (last_costmap_received_.data[i]
				< costmap_obstacle_treshold_) { // add trshold
			relevant_obstacles_.data.push_back(0);
		} else {
			// calculate cell in 2D space where robot is is point (0, 0)
			tf2::Vector3 cell_from_robot;
			int cell_col = (i % (int) (last_costmap_received_.info.width));
			int cell_row = (i / (int) (last_costmap_received_.info.width));

			tf2::Vector3 distance_abs;
			double pixel_size = (double) last_costmap_received_.info.resolution;
			distance_abs.setX(
					((double) cell_col
							- (double) last_costmap_received_.info.width / 2.0)
							* pixel_size);
			distance_abs.setY(
					((double) cell_row
							- (double) last_costmap_received_.info.width / 2.0)
							* pixel_size);
			distance_abs.setZ(0);

			tf2::Vector3 distance_relative;
			tf2::Quaternion lookat_quat;
			lookat_quat.setRPY(0.0, 0.0, robot_angle_);
			tf2::Matrix3x3 rotation_diff(lookat_quat);
			distance_relative = rotation_diff.inverse() * distance_abs;

			tf2::Vector3 distance_lookat;
			lookat_quat.setRPY(0.0, 0.0, collision_look_angle_);
			rotation_diff.setRotation(lookat_quat);
			distance_lookat = rotation_diff.inverse() * distance_abs;

			cur_obstacle_relevant = false;
			if (fabs(robot_twist_angular_.z) < 0.01) {
				if (robot_twist_linear_.x < 0) {
					if (fabs(distance_relative.getY()) < collision_radious_
							&& distance_relative.getX() < 0) {
						cur_obstacle_relevant = true;
					}
				} else {
					if (fabs(distance_relative.getY()) < collision_radious_
							&& distance_relative.getX() > 0) {
						cur_obstacle_relevant = true;
					}
				}
			} else if (robot_twist_angular_.z > 0) {
				if (robot_twist_linear_.x < 0) {
					if (distance_relative.getY() < collision_radious_
							&& distance_lookat.getY() < collision_radious_
							&& distance_relative.getX() < 0) {
						cur_obstacle_relevant = true;
					}
				} else {
					if (distance_relative.getY() > -collision_radious_
							&& distance_lookat.getY() < collision_radious_
							&& distance_relative.getX() > 0) {
						cur_obstacle_relevant = true;
					}
				}
			} else {
				if (robot_twist_linear_.x < 0) {
					if (distance_relative.getY() > -collision_radious_
							&& distance_lookat.getY() > -collision_radious_
							&& distance_relative.getX() < 0) {
						cur_obstacle_relevant = true;
					}
				} else {
					if (distance_relative.getY() < collision_radious_
							&& distance_lookat.getY() > -collision_radious_
							&& distance_relative.getX() > 0) {
						cur_obstacle_relevant = true;
					}
				}
			}

			if (cur_obstacle_relevant) {
				//relevant obstacle in tube found
				//ROS_DEBUG_STREAM_NAMED("obstacleHandler", "[cob_collision_velocity_filter] Detected an obstacle");
				relevant_obstacles_.data.push_back(100);
				double obstacle_distance = getNormXY(distance_abs.getX(), distance_abs.getY());
				if(nearest_obstacle_distance_tmp > obstacle_distance && robot_twist_linear_.x * distance_relative.getX() > 0){
					nearest_obstacle_distance_tmp = obstacle_distance;
					if(distance_relative.getX() > 0){
						nearest_obstacle_direction_ = 1;
					}else{
						nearest_obstacle_direction_ = -1;
					}
					if(nearest_obstacle_distance_tmp < slowdown_radious_ - 0.001){
						nearest_obstacle_distance_ = nearest_obstacle_distance_tmp;
					}
				}
			} else {
				relevant_obstacles_.data.push_back(0);
			}
		}
	}
//	pthread_mutex_unlock(&m_mutex);
//  topic_pub_relevant_obstacles_.publish(last_costmap_received_);
	topic_pub_relevant_obstacles_.publish(relevant_obstacles_);
	if(costmap_received_ || nearest_obstacle_distance_ < nearest_obstacle_linear_){
		nearest_obstacle_linear_ = nearest_obstacle_distance_;
	}
	costmap_received_ = false;
}

double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a,
		geometry_msgs::Point b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double CollisionVelocityFilter::getNormXY(double x, double y) {
	return sqrt( x * x + y * y);
}

double CollisionVelocityFilter::sign(double x) {
	if (x >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle,
		double y_obstacle) {
//  if(x_obstacle<footprint_front_ && x_obstacle>footprint_rear_ && y_obstacle>footprint_right_ && y_obstacle<footprint_left_) {
//    ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
//    return false;
//  }

	return true;
}

void CollisionVelocityFilter::stopMovement() {
	/*
	 geometry_msgs::Twist stop_twist;
	 stop_twist.linear.x = 0.0f; stop_twist.linear.y = 0.0f; stop_twist.linear.z = 0.0f;
	 stop_twist.angular.x = 0.0f; stop_twist.angular.y = 0.0f; stop_twist.linear.z = 0.0f;
	 topic_pub_command_.publish(stop_twist);
	 vx_last_ = 0.0;
	 vy_last_ = 0.0;
	 vtheta_last_ = 0.0;
	 */
}

//#######################
//#### main programm ####
int main(int argc, char** argv) {
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_collision_velocity_filter");

	// create nodeClass
	CollisionVelocityFilter collisionVelocityFilter;

	//set logger level
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
			ros::console::levels::Debug)) {
		ros::console::notifyLoggerLevelsChanged();
	}

	ros::spin();

	return 0;
}

