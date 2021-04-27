#include <ros/ros.h>

// MoveIt libs
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PandaRobot {
private:
    moveit::planning_interface::MoveGroupInterface move_group_interface;

    static void open_gripper(trajectory_msgs::JointTrajectory &posture) {
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        posture.points.resize(1);
        posture.points[0].positions.resize([0.oitons[0] = 0.04;
        posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

    static void close_gripper(trajectory_msgs::JointTrajectory &posture) {
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0;
        posture.points[0].positions[1] = 0;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

public:
    PandaRobot() : move_group_interface("panda_arm") {
        ROS_INFO_STREAM("End-effector link: " << move_group_interface.getEndEffectorLink());

        ros::WallDuration(1.0).sleep();
        move_group_interface.setPlanningTime(45.0);
    }

    void take_pencil() {
//        std::vector<moveit_msgs::Grasp> grasps;
//        grasps.resize(1);
//        grasps[0].grasp_pose.header.frame_id = "panda_link0";
//
//        tf2::Quaternion orientation;
//        orientation.setRPY(-M_PI_2, -M_PI_4, -M_PI_2);
//
//        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//        grasps[0].grasp_pose.pose.position.x = 0.415;
//        grasps[0].grasp_pose.pose.position.y = 0;
//        grasps[0].grasp_pose.pose.position.z = 0.5;
//
//        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
//        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//        grasps[0].pre_grasp_approach.min_distance = 0.095;
//        grasps[0].pre_grasp_approach.desired_distance = 0.115;
//
//        grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
//        grasps[0].post_grasp_retreat.direction.vector.x = -1.0;
//        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//        grasps[0].post_grasp_retreat.min_distance = 0.1;
//        grasps[0].post_grasp_retreat.desired_distance = 0.25;
//
//        open_gripper(grasps[0].grasp_posture);
//        close_gripper(grasps[0].grasp_posture);
//
//        move_group_interface.setSupportSurfaceName("table");
//        move_group_interface.pick("pencil", grasps);
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = 0.28;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.5;

        move_group_interface.setPoseTarget(target_pose1);
        move_group_interface.move();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    panda.take_pencil();
    return 0;
}