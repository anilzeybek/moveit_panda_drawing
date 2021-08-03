#include "letter_poses.h"
#include <ros/ros.h>

// MoveIt libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <trajectory_msgs/JointTrajectory.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PandaRobot {
private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface hand;
    geometry_msgs::PoseStamped initial_pencil_pose;

    void open_gripper() {
        std::vector<double> open_gripper_joints = {0.04, 0.04};
        hand.setJointValueTarget(open_gripper_joints);
        hand.move();
    }

    void close_gripper() {
        std::vector<double> close_gripper_joints = {0, 0};
        hand.setJointValueTarget(close_gripper_joints);
        hand.move();
    }

    static moveit_msgs::CollisionObject
    add_object(const std::string &name, int type, const std::vector<double> &dimensions,
               const std::vector<double> &positions) {
        moveit_msgs::CollisionObject object;

        object.id = name;
        object.header.frame_id = "panda_link0";

        object.primitives.resize(1);
        if (type == 0) {
            // type=0 => box
            object.primitives[0].type = object.primitives[0].BOX;
            object.primitives[0].dimensions.resize(3);
            object.primitives[0].dimensions[0] = dimensions[0];
            object.primitives[0].dimensions[1] = dimensions[1];
            object.primitives[0].dimensions[2] = dimensions[2];
        } else {
            // else => cylinder
            object.primitives[0].type = object.primitives[0].CYLINDER;
            object.primitives[0].dimensions.resize(2);
            object.primitives[0].dimensions[0] = dimensions[0];
            object.primitives[0].dimensions[1] = dimensions[1];
        }

        object.primitive_poses.resize(1);
        object.primitive_poses[0].position.x = positions[0];
        object.primitive_poses[0].position.y = positions[1];
        object.primitive_poses[0].position.z = positions[2];

        object.operation = object.ADD;
        return object;
    }

public:
    PandaRobot() : arm("panda_arm"), hand("hand") {
        arm.setMaxVelocityScalingFactor(1);
        arm.setMaxAccelerationScalingFactor(1);

        arm.setPlanningTime(3.0);
        arm.rememberJointValues("ready");
    }

    void add_objects() {
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        std::vector<double> dimensions{1, 1, 1};
        std::vector<double> positions{0, 1, 0};
        collision_objects.push_back(add_object("table", 0, dimensions, positions));

        dimensions = {0.10, 0.0035};
        positions = {0, 0.65, 0.55};
        collision_objects.push_back(add_object("pencil", 1, dimensions, positions));

        dimensions = {1, 1, 1};
        positions = {0, -1, 0.5};
        collision_objects.push_back(add_object("board", 0, dimensions, positions));

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    void pick_pencil() {
        open_gripper();

        geometry_msgs::Pose target_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(M_PI_2, M_PI_4, -M_PI);
        target_pose.orientation = tf2::toMsg(orientation);
        auto pencil_pose = planning_scene_interface.getObjectPoses({"pencil"}).at("pencil");

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

    void draw(char letter) {
        auto target_poses = LetterPoses::get_poses(letter);
        for (const auto &target : target_poses) {
            arm.setPoseTarget(target);
            arm.move();
        }
    }

    void place_pencil() {
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
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    panda.add_objects();
    panda.pick_pencil();
    panda.draw('K');
    panda.place_pencil();

    return 0;
}