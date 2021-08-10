#include "PandaRobot.h"

void PandaRobot::open_gripper() {
    std::vector<double> open_gripper_joints = {0.04, 0.04};
    hand.setJointValueTarget(open_gripper_joints);
    hand.move();
}

void PandaRobot::close_gripper() {
    std::vector<double> close_gripper_joints = {0, 0};
    hand.setJointValueTarget(close_gripper_joints);
    hand.move();
}

void PandaRobot::initialize_markers(const std::string &word) {
    for (int i = 0; i < word.size(); i++) {
        visualization_msgs::Marker line_strip;

        line_strip.header.frame_id = "panda_link0";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "line";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = i;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.01;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;
        line_strip.lifetime = ros::Duration(0);

        line_strips.push_back(line_strip);
    }
}

PandaRobot::PandaRobot() : arm("panda_arm"), hand("hand") {
    arm.setMaxVelocityScalingFactor(0.8);
    arm.setMaxAccelerationScalingFactor(0.8);

    arm.setPlanningTime(3.0);
    arm.rememberJointValues("ready");

    LetterPoses::generate_point_matrix();

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void PandaRobot::pick_pencil(geometry_msgs::Pose pencil_pose) {
    open_gripper();

    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(M_PI_2, M_PI_4, -M_PI);
    target_pose.orientation = tf2::toMsg(orientation);

    target_pose.position.x = pencil_pose.position.x;
    target_pose.position.y = pencil_pose.position.y - 0.10;
    target_pose.position.z = pencil_pose.position.z + 0.04;
    arm.setPoseTarget(target_pose);
    arm.move();

    arm.setSupportSurfaceName("table");
    hand.attachObject("pencil", "panda_hand", {"panda_leftfinger", "panda_rightfinger"});
    close_gripper();

    this->initial_pencil_pose = arm.getCurrentPose();
}

void PandaRobot::place_pencil() {
    this->initial_pencil_pose.pose.position.z += 0.01;
    arm.setPoseTarget(this->initial_pencil_pose);
    arm.move();

    ros::Duration(0.5).sleep();
    open_gripper();
    hand.detachObject("pencil");
    
    // z ekseni yukarı
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose w = arm.getCurrentPose().pose;
    w.position.z += 0.02;

    moveit_msgs::RobotTrajectory trajectory;
    arm.computeCartesianPath(waypoints, 0.005, 0.000, trajectory);
    arm.execute(trajectory);

    arm.setNamedTarget("ready");
    arm.move();

    close_gripper();
}

void PandaRobot::draw_letter(char letter, visualization_msgs::Marker &line_strip) {
    auto target_poses = LetterPoses::get_poses(letter);
    for (const auto &target : target_poses) {
        arm.setPoseTarget(target);
        arm.move();

        line_strip.points.push_back(arm.getCurrentPose().pose.position);
        if (line_strip.points.size() >= 2)
            marker_pub.publish(line_strip);
    }
}

void PandaRobot::draw_word(const std::string &word) {
    initialize_markers(word);

    for (int i = 0; i < word.size(); i++) {
        draw_letter(word[i], line_strips[i]);
        LetterPoses::increase_x_index();
    }
}

void PandaRobot::draw_sentence(const std::string &sentence) {
}

void PandaRobot::go_pose(geometry_msgs::Pose target) {
    arm.setPoseTarget(target);
    arm.move();
}
