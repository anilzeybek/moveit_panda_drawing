#ifndef PANDAROBOT_H
#define PANDAROBOT_H

#include "Environment.h"
#include "LetterPoses.h"
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

    void put_space();

public:
    PandaRobot();

    void go_pose(geometry_msgs::Pose target);

    void pick_pencil(geometry_msgs::Pose pencil_pose);

    void place_pencil();

    void draw_letter(char letter);

    void draw_word(std::string word);

    void draw_sentence(std::string sentence);
};

#endif