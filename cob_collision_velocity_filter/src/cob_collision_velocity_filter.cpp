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
CollisionVelocityFilter::CollisionVelocityFilter()
{
  // create node handle
  nh_ = ros::NodeHandle("~");
  initTfListener();
  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  // node handle to get footprint from parameter server
  std::string costmap_parameter_source;
  if(!nh_.hasParam("costmap_parameter_source")) ROS_WARN("Checking default source [/local_costmap_node/costmap] for costmap parameters");
  nh_.param("costmap_parameter_source",costmap_parameter_source, std::string("/local_costmap_node/costmap"));

  ros::NodeHandle local_costmap_nh_(costmap_parameter_source);

  nh_.param("costmap_obstacle_treshold", costmap_obstacle_treshold_, 50);

  // implementation of topics to publish (command for base and list of relevant obstacles)
  topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
  topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::OccupancyGrid>("relevant_obstacles_grid", 1);
  topic_pub_lookat_ = nh_.advertise<geometry_msgs::PoseStamped>("collision_lookat", 1);
  // subscribe to twist-movement of teleop
  joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("teleop_twist", 1, boost::bind(&CollisionVelocityFilter::joystickVelocityCB, this, _1));
  // subscribe to the costmap to receive inflated cells
  obstacles_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("obstacles", 1, boost::bind(&CollisionVelocityFilter::obstaclesCB, this, _1));

  // read parameters from parameter server
  // parameters from costmap
  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/global_frame")) ROS_WARN("Used default parameter for global_frame [/map]");
  local_costmap_nh_.param(costmap_parameter_source+"/global_frame", global_frame_, std::string("/map"));

  if(!local_costmap_nh_.hasParam(costmap_parameter_source+"/robot_base_frame")) ROS_WARN("Used default parameter for robot_frame [/base_link]");
  local_costmap_nh_.param(costmap_parameter_source+"/robot_base_frame", robot_frame_, std::string("/base_link"));

  if(!nh_.hasParam("slowdown_radious")) ROS_WARN("Used default parameter for slowdown_radious [1.5]");
  nh_.param("slowdown_radious", slowdown_radious_, 1.5);\
  if(slowdown_radious_ < 0.2){
	slowdown_radious_ = 0.2; //TODO: parametarize
  }
  if(!nh_.hasParam("stop_radious")) ROS_WARN("Used default parameter for stop_radious [1.0]");
  nh_.param("stop_radious", stop_radious_, 1.0);
  if(stop_radious_ < slowdown_radious_ - 0.1){
	stop_radious_ = slowdown_radious_ - 0.1; //TODO: parametarize
  }
  if(!nh_.hasParam("collision_radious")) ROS_WARN("Used default parameter for collision_radious [0.5]");
  nh_.param("collision_radious", collision_radious_, 0.5);
  if(collision_radious_ < 0.1){
	  collision_radious_ = 0.1; //TODO: parametarize
  }
  if(!nh_.hasParam("vx_max")) ROS_WARN("Used default parameter for vx_max [1.0]");
  nh_.param("vx_max", vx_max_, 1.0);
  braking_distance_ = slowdown_radious_ - stop_radious_;
  desired_stopping_time_ = (2.0 * braking_distance_)/vx_max_;
  breaking_accel_ = vx_max_ / desired_stopping_time_;
  stopping_interval_ = desired_stopping_time_;

  last_time_ = ros::Time::now();
  vx_last_ = 0.0;
}

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter(){}

void CollisionVelocityFilter::initTfListener(){
	double cache_time_ = 3.0;
	tfBuffer_ptr.reset(new tf2_ros::Buffer(ros::Duration(cache_time_), true));
	tfListener_ptr.reset(new tf2_ros::TransformListener(*tfBuffer_ptr));
	sleep(cache_time_);
}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist){
  pthread_mutex_lock(&m_mutex);
  robot_twist_linear_ = twist->linear;
  robot_twist_angular_ = twist->angular;
  pthread_mutex_unlock(&m_mutex);

  //get robot direction

  if(tfBuffer_ptr->canTransform(global_frame_, robot_frame_, ros::Time(0))){
    tf_robot_ = tfBuffer_ptr->lookupTransform(global_frame_, robot_frame_, ros::Time(0));
    tf2::Quaternion robot_quat(tf_robot_.transform.rotation.x, tf_robot_.transform.rotation.y, tf_robot_.transform.rotation.z, tf_robot_.transform.rotation.w);
	geometry_msgs::Vector3 robot_euler;
	tf2::Matrix3x3(robot_quat).getEulerYPR(robot_euler.z, robot_euler.y, robot_euler.x);
	robot_angle_ = robot_euler.z;
	robot_quat.setRPY(0.0, 0.0, robot_euler.z);
	if(fabs(robot_twist_linear_.x) < 0.001){
	  collision_look_angle_ = robot_angle_;
	}else if(robot_twist_linear_.x > 0.0){
		  collision_look_angle_ = robot_angle_ + ( stopping_interval_ * robot_twist_angular_.z);
	}else{
	  collision_look_angle_ = - M_PI + robot_angle_ + ( stopping_interval_ * robot_twist_angular_.z);
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
  }else{
    ROS_INFO("Can't find origin TF [%s]", robot_frame_.c_str());
  }
  // check for relevant obstacles
  obstacleHandler();
  // stop if we are about to run in an obstacle
  performControllerStep();

}

// obstaclesCB reads obstacles from costmap
void CollisionVelocityFilter::obstaclesCB(const nav_msgs::OccupancyGrid::ConstPtr &obstacles){
  pthread_mutex_lock(&m_mutex);
  if(obstacles->data.size()!=0) costmap_received_ = true;
  last_costmap_received_ = * obstacles;
  pthread_mutex_unlock(&m_mutex);
}


// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep() {


  double vx_max, vy_max;
  geometry_msgs::Twist cmd_vel,cmd_vel_in;

  cmd_vel_in.linear = robot_twist_linear_;
  cmd_vel_in.angular = robot_twist_angular_;

  cmd_vel.linear = robot_twist_linear_;
  cmd_vel.angular = robot_twist_angular_;
  /*
  ros::Duration dt_ros = ros::Time::now() - last_time_;
  last_time_ = ros::Time::now();
  double dt = dt_ros.toSec();

  double vel_angle = atan2(cmd_vel.linear.y,cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max > fabs(cmd_vel.linear.x)) vx_max = fabs(cmd_vel.linear.x);
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max > fabs(cmd_vel.linear.y)) vy_max = fabs(cmd_vel.linear.y);

  //Slow down in any way while approximating an obstacle:
  if(closest_obstacle_dist_ < influence_radius_) {
    double F_x, F_y;
    double vx_d, vy_d, vx_factor, vy_factor;
    double kv_obst=kv_, vx_max_obst=vx_max, vy_max_obst=vy_max;

    //implementation for linear decrease of v_max:
    double obstacle_linear_slope_x = vx_max / (obstacle_damping_dist_ - stop_threshold_);
    vx_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_x;
    if(vx_max_obst > vx_max) vx_max_obst = vx_max;
    else if(vx_max_obst < 0.0f) vx_max_obst = 0.0f;

    double obstacle_linear_slope_y = vy_max / (obstacle_damping_dist_ - stop_threshold_);
    vy_max_obst = (closest_obstacle_dist_- stop_threshold_ + stop_threshold_/10.0f) * obstacle_linear_slope_y;
    if(vy_max_obst > vy_max) vy_max_obst = vy_max;
    else if(vy_max_obst < 0.0f) vy_max_obst = 0.0f;

    //Translational movement
    //calculation of v factor to limit maxspeed
    double closest_obstacle_dist_x = closest_obstacle_dist_ * cos(closest_obstacle_angle_);
    double closest_obstacle_dist_y = closest_obstacle_dist_ * sin(closest_obstacle_angle_);
    vx_d = kp_/kv_obst * closest_obstacle_dist_x;
    vy_d = kp_/kv_obst * closest_obstacle_dist_y;
    vx_factor = vx_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    vy_factor = vy_max_obst / sqrt(vy_d*vy_d + vx_d*vx_d);
    if(vx_factor > 1.0) vx_factor = 1.0;
    if(vy_factor > 1.0) vy_factor = 1.0;

    F_x = - kv_obst * vx_last_ + vx_factor * kp_ * closest_obstacle_dist_x;
    F_y = - kv_obst * vy_last_ + vy_factor * kp_ * closest_obstacle_dist_y;

    cmd_vel.linear.x = vx_last_ + F_x / virt_mass_ * dt;
    cmd_vel.linear.y = vy_last_ + F_y / virt_mass_ * dt;
    ROS_DEBUG("CollisionVelocityFilter::performControllerStep - %f -> %f", cmd_vel_in.linear.x, cmd_vel.linear.x);

  }

  // make sure, the computed and commanded velocities are not greater than the specified max velocities
  if (fabs(cmd_vel.linear.x) > vx_max) cmd_vel.linear.x = sign(cmd_vel.linear.x) * vx_max;
  if (fabs(cmd_vel.linear.y) > vy_max) cmd_vel.linear.y = sign(cmd_vel.linear.y) * vy_max;
  if (fabs(cmd_vel.angular.z) > vtheta_max_) cmd_vel.angular.z = sign(cmd_vel.angular.z) * vtheta_max_;

  // limit acceleration:
  // only acceleration (in terms of speeding up in any direction) is limited,
  // deceleration (in terms of slowing down) is handeled either by cob_teleop or the potential field
  // like slow-down behaviour above
  if (fabs(cmd_vel.linear.x) > fabs(vx_last_))
  {
    if ((cmd_vel.linear.x - vx_last_)/dt > ax_max_)
      cmd_vel.linear.x = vx_last_ + ax_max_ * dt;
    else if((cmd_vel.linear.x - vx_last_)/dt < -ax_max_)
      cmd_vel.linear.x = vx_last_ - ax_max_ * dt;
  }
  if (fabs(cmd_vel.linear.y) > fabs(vy_last_))
  {
    if ((cmd_vel.linear.y - vy_last_)/dt > ay_max_)
      cmd_vel.linear.y = vy_last_ + ay_max_ * dt;
    else if ((cmd_vel.linear.y - vy_last_)/dt < -ay_max_)
      cmd_vel.linear.y = vy_last_ - ay_max_ * dt;
  }
  if (fabs(cmd_vel.angular.z) > fabs(vtheta_last_))
  {
    if ((cmd_vel.angular.z - vtheta_last_)/dt > atheta_max_)
      cmd_vel.angular.z = vtheta_last_ + atheta_max_ * dt;
    else if ((cmd_vel.angular.z - vtheta_last_)/dt < -atheta_max_)
      cmd_vel.angular.z = vtheta_last_ - atheta_max_ * dt;
  }

  pthread_mutex_lock(&m_mutex);
  vx_last_ = cmd_vel.linear.x;
  vy_last_ = cmd_vel.linear.y;
  vtheta_last_ = cmd_vel.angular.z;
  pthread_mutex_unlock(&m_mutex);

  velocity_limited_marker_.publishMarkers(cmd_vel_in.linear.x, cmd_vel.linear.x, cmd_vel_in.linear.y, cmd_vel.linear.y, cmd_vel_in.angular.z, cmd_vel.angular.z);

  // if closest obstacle is within stop_threshold, then do not move
  if( closest_obstacle_dist_ < stop_threshold_ ) {

	ROS_DEBUG("CollisionVelocityFilter::performControllerStep calls stopMovement()");
    stopMovement();
  }
  else
  {
    // publish adjusted velocity
    topic_pub_command_.publish(cmd_vel);
  }
  */
  topic_pub_command_.publish(cmd_vel);
  return;
}

void CollisionVelocityFilter::obstacleHandler() {
  if(!costmap_received_) {
    ROS_WARN("No costmap has been received by cob_collision_velocity_filter, the robot will drive without obstacle avoidance!");
    return;
  }
  double cur_distance_to_center, cur_distance_to_border;
  double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
  bool cur_obstacle_relevant;
  geometry_msgs::Point cur_obstacle_robot;
  geometry_msgs::Point zero_position;
  zero_position.x=0.0f;
  zero_position.y=0.0f;
  zero_position.z=0.0f;


  //find relevant obstacles
//  pthread_mutex_lock(&m_mutex);
  relevant_obstacles_.header = last_costmap_received_.header;
  relevant_obstacles_.info = last_costmap_received_.info;
  relevant_obstacles_.data.clear();

  for(unsigned int i = 0; i < last_costmap_received_.data.size(); i++) {
    if (last_costmap_received_.data[i] == -1) {
      relevant_obstacles_.data.push_back(-1);
    }
    else if (last_costmap_received_.data[i] < costmap_obstacle_treshold_) { // add trshold
      relevant_obstacles_.data.push_back(0);
    }
    else {
      // calculate cell in 2D space where robot is is point (0, 0)
      tf2::Vector3 cell_from_robot;
      int cell_col = (i%(int)(last_costmap_received_.info.width));
      int cell_row = (i/(int)(last_costmap_received_.info.width));

      tf2::Vector3 distance_abs;
      distance_abs.setX((-((int)last_costmap_received_.info.width/2) + cell_row) * last_costmap_received_.info.resolution);
      distance_abs.setY((-((int)last_costmap_received_.info.width/2) + cell_col) * last_costmap_received_.info.resolution);
      distance_abs.setZ(0);
      if(distance_abs.getX() > 0 && distance_abs.getY() > 0 ){
//    	ROS_WARN("obstacleHandler (x,y) = %f , %f", distance_abs.getX(), distance_abs.getY());
        cur_obstacle_relevant = true;
      }else{
          cur_obstacle_relevant = false;
      }
//      cur_obstacle_relevant = true;
//      geometry_msgs::Point cell;
//      cell.x = (i%(int)(last_costmap_received_.info.width))*last_costmap_received_.info.resolution + last_costmap_received_.info.origin.position.x;
//      cell.y = (i/(int)(last_costmap_received_.info.width))*last_costmap_received_.info.resolution + last_costmap_received_.info.origin.position.y;
//      cell.z = 0.0f;
//
//
//      cur_obstacle_relevant = false;
//      cur_distance_to_center = getDistance2d(zero_position, cell);


		if (cur_obstacle_relevant){
//			ROS_DEBUG_STREAM_NAMED("obstacleHandler", "[cob_collision_velocity_filter] Detected an obstacle");
			//relevant obstacle in tube found
			relevant_obstacles_.data.push_back(100);

			/*
			//now calculate distance of current, relevant obstacle to robot
			if (obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left){
				//obstacle in front:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
			}else if (obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left){
				//obstacle left:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
			}else if (obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right){
				//obstacle in rear:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
			}else{
				//obstacle right:
				cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
			}

			if (cur_distance_to_border < closest_obstacle_dist_){
				closest_obstacle_dist_ = cur_distance_to_border;
				closest_obstacle_angle_ = obstacle_theta_robot;
			}
			*/
		}else{
			  relevant_obstacles_.data.push_back(0);
		}

    }
  }
//  pthread_mutex_unlock(&m_mutex);
//  topic_pub_relevant_obstacles_.publish(last_costmap_received_);
  topic_pub_relevant_obstacles_.publish(relevant_obstacles_);

}

// load robot footprint from costmap_2d_ros to keep same footprint format
std::vector<geometry_msgs::Point> CollisionVelocityFilter::loadRobotFootprint(ros::NodeHandle node){
  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;
  double padding;

  std::string padding_param, footprint_param;
  if(!node.searchParam("footprint_padding", padding_param))
    padding = 0.01;
  else
    node.param(padding_param, padding, 0.01);

  //grab the footprint from the parameter server if possible
  XmlRpc::XmlRpcValue footprint_list;
  std::string footprint_string;
  std::vector<std::string> footstring_list;
  if(node.searchParam("footprint", footprint_param)){
    node.getParam(footprint_param, footprint_list);
    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      footprint_string = std::string(footprint_list);

      //if there's just an empty footprint up there, return
      if(footprint_string == "[]" || footprint_string == "")
        return footprint;

      boost::erase_all(footprint_string, " ");

      boost::char_separator<char> sep("[]");
      boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
      footstring_list = std::vector<std::string>(tokens.begin(), tokens.end());
    }
    //make sure we have a list of lists
    if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) && !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5)){
      ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", footprint_param.c_str(), std::string(footprint_list).c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for(int i = 0; i < footprint_list.size(); ++i){
        //make sure we have a list of lists of size 2
        XmlRpc::XmlRpcValue point = footprint_list[i];
        if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
          ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
        }

        //make sure that the value we're looking at is either a double or an int
        if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
        pt.x += sign(pt.x) * padding;

        //make sure that the value we're looking at is either a double or an int
        if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
          ROS_FATAL("Values in the footprint specification must be numbers");
          throw std::runtime_error("Values in the footprint specification must be numbers");
        }
        pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
        pt.y += sign(pt.y) * padding;

        footprint.push_back(pt);

        node.deleteParam(footprint_param);
        std::ostringstream oss;
        bool first = true;
        BOOST_FOREACH(geometry_msgs::Point p, footprint) {
          if(first) {
            oss << "[[" << p.x << "," << p.y << "]";
            first = false;
          }
          else {
            oss << ",[" << p.x << "," << p.y << "]";
          }
        }
        oss << "]";
        node.setParam(footprint_param, oss.str().c_str());
        node.setParam("footprint", oss.str().c_str());
      }
    }

    else if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
      std::vector<geometry_msgs::Point> footprint_spec;
      bool valid_foot = true;
      BOOST_FOREACH(std::string t, footstring_list) {
        if( t != "," ) {
          boost::erase_all(t, " ");
          boost::char_separator<char> pt_sep(",");
          boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
          std::vector<std::string> point(pt_tokens.begin(), pt_tokens.end());

          if(point.size() != 2) {
            ROS_WARN("Each point must have exactly 2 coordinates");
            valid_foot = false;
            break;
          }

          std::vector<double>tmp_pt;
          BOOST_FOREACH(std::string p, point) {
            std::istringstream iss(p);
            double temp;
            if(iss >> temp) {
              tmp_pt.push_back(temp);
            }
            else {
              ROS_WARN("Each coordinate must convert to a double.");
              valid_foot = false;
              break;
            }
          }
          if(!valid_foot)
            break;

          geometry_msgs::Point pt;
          pt.x = tmp_pt[0];
          pt.y = tmp_pt[1];

          footprint_spec.push_back(pt);
        }
      }
      if (valid_foot) {
        footprint = footprint_spec;
        node.setParam("footprint", footprint_string);
      }
      else {
        ROS_FATAL("This footprint is not vaid it must be specified as a list of lists with at least 3 points, you specified %s", footprint_string.c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }
    }
  }
/*
  footprint_right_ = 0.0f; footprint_left_ = 0.0f; footprint_front_ = 0.0f; footprint_rear_ = 0.0f;
  //extract rectangular borders for simplifying:
  for(unsigned int i=0; i<footprint.size(); i++) {
    if(footprint[i].x > footprint_front_) footprint_front_ = footprint[i].x;
    if(footprint[i].x < footprint_rear_) footprint_rear_ = footprint[i].x;
    if(footprint[i].y > footprint_left_) footprint_left_ = footprint[i].y;
    if(footprint[i].y < footprint_right_) footprint_right_ = footprint[i].y;
  }
  ROS_DEBUG("Extracted rectangular footprint for cob_collision_velocity_filter: Front: %f, Rear %f, Left: %f, Right %f", footprint_front_, footprint_rear_, footprint_left_, footprint_right_);
*/
  return footprint;
}

double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt( pow(a.x - b.x,2) + pow(a.y - b.y,2) );
}

double CollisionVelocityFilter::sign(double x) {
  if(x >= 0.0f) return 1.0f;
  else return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle, double y_obstacle) {
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
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_collision_velocity_filter");

  // create nodeClass
  CollisionVelocityFilter collisionVelocityFilter;

  //set logger level
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  ros::spin();

  return 0;
}

