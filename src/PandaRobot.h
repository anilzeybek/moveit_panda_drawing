#ifndef PANDAROBOT_H
#define PANDAROBOT_H

#include "Environment.h"
#include "LetterPoses.h"
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>

class PandaRobot {
private:
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface hand;
    geometry_msgs::PoseStamped initial_pencil_pose;

    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    std::vector<visualization_msgs::Marker> line_strips;

    void open_gripper();

    void close_gripper();

    void put_space();

    void initialize_markers(const std::string& word);

public:
    PandaRobot();

    void go_pose(geometry_msgs::Pose target);

    void pick_pencil(geometry_msgs::Pose pencil_pose);

    void place_pencil();

    void draw_letter(char letter, visualization_msgs::Marker &line_strip);

    void draw_word(const std::string& word);

    void draw_sentence(const std::string& sentence);
};

#endif