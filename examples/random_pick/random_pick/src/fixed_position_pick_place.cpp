/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture, std::vector<std::string>& finger_joints, 
                                                                     std::vector<double>& open_dist)
{
  /* Add both finger joints of hitbot gripper. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = finger_joints[0];
  posture.joint_names[1] = finger_joints[1];

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = open_dist[0];
  posture.points[0].positions[1] = open_dist[1];
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture, std::vector<std::string>& finger_joints)
{
  /* Add both finger joints of hitbot gripper. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = finger_joints[0];
  posture.joint_names[1] = finger_joints[1];

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0;
  posture.points[0].positions[1] = 0.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, std::string& frame_id,
                                                    std::string& support_surface, 
                                                    std::vector<std::string>& finger_joints, 
                                                    std::vector<double>& open_dists,
                                                    std::vector<double>& grasp_position,
                                                    std::vector<double>& grasp_orientation)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // This is the pose of tool0.
  grasps[0].grasp_pose.header.frame_id = frame_id;
  tf2::Quaternion orientation;
  orientation.setRPY(grasp_orientation[0], grasp_orientation[1], grasp_orientation[2]);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = grasp_position[0];
  grasps[0].grasp_pose.pose.position.y = grasp_position[1];
  grasps[0].grasp_pose.pose.position.z = grasp_position[2];

  // Setting pre-grasp approach
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = frame_id;
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.10;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = frame_id;
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.10;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture, finger_joints, open_dists);

  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture, finger_joints);

  // Set support surface as table1.
  move_group.setSupportSurfaceName(support_surface);
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group, std::string& frame_id, 
                                                std::string& support_surface, 
                                                std::vector<std::string>& finger_joints, 
                                                std::vector<double>& open_dists, 
                                                std::vector<double>& place_position,
                                                std::vector<double>& place_orientation)
{
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = frame_id;
  tf2::Quaternion orientation;
  orientation.setRPY(place_orientation[0], place_orientation[1], place_orientation[2]);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = place_position[0];
  place_location[0].place_pose.pose.position.y = place_position[1];
  place_location[0].place_pose.pose.position.z = place_position[2];

  // Setting pre-place approach
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = frame_id;
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.05;
  place_location[0].pre_place_approach.desired_distance = 0.10;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = frame_id;
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.05;
  place_location[0].post_place_retreat.desired_distance = 0.10;

  // Setting posture of eef after placing object
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture, finger_joints, open_dists);

  // Set support surface as table2.
  group.setSupportSurfaceName("operation_surface");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                                          std::string& frame_id, std::vector<double>& object_position)
{
  // Creating Environment
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = frame_id;
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
  collision_objects[0].primitives[0].dimensions.resize(1);
  collision_objects[0].primitives[0].dimensions[0] = 0.04;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = object_position[0];
  collision_objects[0].primitive_poses[0].position.y = object_position[1];
  collision_objects[0].primitive_poses[0].position.z = object_position[2];

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

std::vector<std::string> split_by_space(std::string& ss)
{
  std::istringstream buf(ss);
  std::istream_iterator<std::string> beg(buf), end;
  std::vector<std::string> vs(beg, end);
  return vs;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cobotta_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Get the parameter values from the ros_param server */
  std::string frame_id;
  std::string support_surface;
  std::string group_name;
  std::string finger_joints;
  std::string finger_open_dists;
  std::string object_xyz;     
  std::string grasp_xyz;
  std::string grasp_rpy;
  std::string place_xyz;
  std::string place_rpy;                                                                      
  nh.param<std::string>(ros::this_node::getName()+"/frame_id", frame_id, "base");
  nh.param<std::string>(ros::this_node::getName()+"/frame_id", support_surface, "operation_surface");
  nh.param<std::string>(ros::this_node::getName()+"/group_name", group_name, "ur5_arm");
  nh.param<std::string>(ros::this_node::getName()+"/finger_joints", finger_joints, 
                            "hitbot_base_finger0_joint hitbot_base_finger1_joint");
  nh.param<std::string>(ros::this_node::getName()+"/finger_open_dists", finger_open_dists, "-0.01 0.01");
  nh.param<std::string>(ros::this_node::getName()+"/object_xyz", object_xyz, "0.107 -0.545 -0.10"); 
  nh.param<std::string>(ros::this_node::getName()+"/grasp_xyz", grasp_xyz, "0.107 -0.545 0.108");                                                                           
  nh.param<std::string>(ros::this_node::getName()+"/grasp_rpy", grasp_rpy, "3.1415 0.0 -0.7853");                                                                           
  nh.param<std::string>(ros::this_node::getName()+"/place_xyz", place_xyz, "-0.107 -0.545 -0.10");                                                                           
  nh.param<std::string>(ros::this_node::getName()+"/place_rpy", place_rpy, "0.0 0.0 0.0"); 

  /* Get the finger joints name */
  std::vector<std::string> finger_joint_vect = split_by_space(finger_joints); 

  /* Get the finger joints open dists */
  std::vector<std::string> ss_tmp = split_by_space(finger_open_dists);
  std::vector<double> finger_open_dist_vect;
  for(auto& s: ss_tmp)
    finger_open_dist_vect.push_back(std::stod(s));

  /* Get the object position xyz */
  ss_tmp = split_by_space(object_xyz);
  ROS_INFO_STREAM("object_xyz: " << object_xyz);
  std::vector<double> object_position;
  for(auto& s: ss_tmp)
    object_position.push_back(std::stod(s));

  /* Get the grasp position xyz */
  ss_tmp = split_by_space(grasp_xyz);
  std::vector<double> grasp_position;
  for(auto& s: ss_tmp)
    grasp_position.push_back(std::stod(s));

  /* Get the grasp orientation rpy */
  ss_tmp = split_by_space(grasp_rpy);
  std::vector<double> grasp_orientation;
  for(auto& s: ss_tmp)
    grasp_orientation.push_back(std::stod(s));

  /* Get the place position xyz */
  ss_tmp = split_by_space(place_xyz);
  std::vector<double> place_position;
  for(auto& s: ss_tmp)
    place_position.push_back(std::stod(s));

  /* Get the place orientation rpy */
  ss_tmp = split_by_space(place_rpy);
  std::vector<double> place_orientation;
  for(auto& s: ss_tmp)
    place_orientation.push_back(std::stod(s));

  /* Create move group */
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group(group_name);
  group.setPlanningTime(45.0);

  /* Add collision object */
  addCollisionObjects(planning_scene_interface, frame_id, object_position);

  /* Move to the home pose */
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.setStartStateToCurrentState();
  group.setNamedTarget("home");
  if (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    if (group.execute(my_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return 1;
  }

  /* Wait a bit for ROS things to initialize */
  ros::WallDuration(1.0).sleep();

  pick(group, frame_id, support_surface, finger_joint_vect, finger_open_dist_vect, grasp_position, grasp_orientation);

  ros::WallDuration(1.0).sleep();

  place(group, frame_id, support_surface, finger_joint_vect, finger_open_dist_vect, place_position, place_orientation);

  ros::waitForShutdown();
  return 0;
}
