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

void PandaRobot::put_space() {
}

PandaRobot::PandaRobot() : arm("panda_arm"), hand("hand") {
    arm.setMaxVelocityScalingFactor(0.8);
    arm.setMaxAccelerationScalingFactor(0.8);

    arm.setPlanningTime(3.0);
    arm.rememberJointValues("ready");
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

    arm.setNamedTarget("ready");
    arm.move();

    close_gripper();
}

void PandaRobot::draw_letter(char letter) {
    auto target_poses = LetterPoses::get_poses(letter);
    for (const auto &target : target_poses) {
        arm.setPoseTarget(target);
        arm.move();
    }
}

void PandaRobot::draw_word(std::string word) {
}

void PandaRobot::draw_sentence(std::string sentence) {
}

void PandaRobot::go_pose(geometry_msgs::Pose target) {
    arm.setPoseTarget(target);
    arm.move();
}
