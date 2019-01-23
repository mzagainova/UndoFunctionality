/*
 * pause_node.cc
 * Copyright (c) 2018, Michael Simmons, David Feil-Seifer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Nevada, Reno nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <pause_pkg/Move.h>
#include <pause_pkg/Plan.h>
#include <pause_pkg/Stop.h>


/*
  reset_
    In RVIZ resets the robot to a default position
    x
    y
    z
    w
  returns:
    true: if successful
    false: not successful
*/

bool reset_(pause_pkg::Stop::Request &req,
            pause_pkg::Stop::Response &res)
{
    moveit::planning_interface::MoveGroup group("right_arm");
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 0;
    target_pose.position.x = 0;
    target_pose.position.y = 0;
    target_pose.position.z = 0;
    group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success=false;
    success=group.move();

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    res.result = success;
    return success;
}

/*
  plan_
    In RVIZ plans the robots right arm movement
  args: coordinates
    x
    y
    z
    w
  returns:
    true: if successful
    false: not successful
*/
bool plan_(pause_pkg::Plan::Request &req,
           pause_pkg::Plan::Response &res)
{
  moveit::planning_interface::MoveGroup group("right_arm");
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = req.w;
  target_pose.position.x = req.x;
  target_pose.position.y = req.y;
  target_pose.position.z = req.z;
  group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  res.result=success;
  return true;
}
/*
  move_
    In RVIZ moves the robots right arm movement
  args: coordinates
    x
    y
    z
    w
  returns:
    true: if successful
    false: not successful
*/
bool move_(pause_pkg::Move::Request &req,
           pause_pkg::Move::Response &res)
{
  //sleep(2.0);
  moveit::planning_interface::MoveGroup group("right_arm");
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = req.w;
  target_pose.position.x = req.x;
  target_pose.position.y = req.y;
  target_pose.position.z = req.z;
  group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success=false;
  success=group.move();

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  res.result = success;
  return success;
}
/*
  plan_
    In RVIZ stops the robots right arm movement
  args:
    NONE
  returns:
    true: always
*/
bool stop_(pause_pkg::Stop::Request &req,
           pause_pkg::Stop::Response &res)
{
  //ISSUE: DON'T KNOW HOW TO CHECK IF YOU WERE SUCCESSFUL since group.stop is void maybe do an active check
  moveit::planning_interface::MoveGroup group("right_arm");
  group.stop();
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pause_node");
  ros::NodeHandle nh;
  //declare subscribers
  ros::ServiceServer plan_service = nh.advertiseService("plan", plan_);
  ros::ServiceServer move_service = nh.advertiseService("move", move_);
  ros::ServiceServer stop_service = nh.advertiseService("stop", stop_);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //async spinner thread
  ros::AsyncSpinner spinner(4);
  spinner.start();





  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
  }
  return 0;
}
