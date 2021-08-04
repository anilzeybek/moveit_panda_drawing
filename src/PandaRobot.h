#ifndef PANDAROBOT_H
#define PANDAROBOT_H

#include "LetterPoses.h"
#include "Environment.h"
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>


class PandaRobot {
private:
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface hand;
    geometry_msgs::PoseStamped initial_pencil_pose;

    void open_gripper();

    void close_gripper();

public:
    PandaRobot();

    void pick_pencil(geometry_msgs::Pose pencil_pose);

    void draw(char letter);

    void place_pencil();
};

#endif