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
    std::string planning_group = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    ros::NodeHandle n;
    ros::Publisher pub;

    void open_gripper() {
        trajectory_msgs::JointTrajectory msg;
        msg.header.seq = 1;
        msg.header.stamp.sec = 0;
        msg.header.stamp.nsec = 0;
        msg.header.frame_id = "";

        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0.2, 0.2};
        point.velocities = {0.1, 0.1};
        point.accelerations = {0.1, 0.1};
        point.effort = {0.1, 0.1};
        point.time_from_start.sec = 1;
        point.time_from_start.nsec = 0;
        msg.points.push_back(point);

        pub.publish(msg);
        ros::WallDuration(1.0).sleep();
    }

    void close_gripper() {
        trajectory_msgs::JointTrajectory msg;
        msg.header.seq = 1;
        msg.header.stamp.sec = 0;
        msg.header.stamp.nsec = 0;
        msg.header.frame_id = "";

        msg.joint_names = {"panda_finger_joint1", "panda_finger_joint2"};

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {0, 0};
        point.velocities = {0.1, 0.1};
        point.accelerations = {0.1, 0.1};
        point.effort = {0.1, 0.1};
        point.time_from_start.sec = 1;
        point.time_from_start.nsec = 0;
        msg.points.push_back(point);

        pub.publish(msg);
        ros::WallDuration(1.0).sleep();
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
    PandaRobot() : move_group_interface(planning_group) {
        pub = n.advertise<trajectory_msgs::JointTrajectory>("/panda_hand_controller/command", 10, true);

        std::cout << "Joint model group names:" << std::endl;
        for (const auto &i : move_group_interface.getJointModelGroupNames())
            std::cout << i << std::endl;

        std::cout << "Planning group in-use: " << planning_group << std::endl;
        std::cout << "End-effector link: " << move_group_interface.getEndEffectorLink() << std::endl;
        std::cout << "Joint names:" << std::endl;
        for (const auto &i : move_group_interface.getJointNames())
            std::cout << i << std::endl;

//        auto pose = move_group_interface.getCurrentPose();
//        auto rpy = move_group_interface.getCurrentRPY();

        ros::WallDuration(1.0).sleep();
        move_group_interface.setPlanningTime(15.0);

        move_group_interface.rememberJointValues("initial");
    }

    void add_objects() {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

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

    void take_pencil() {
        open_gripper();

        geometry_msgs::Pose target_pose;
        tf2::Quaternion orientation;
        orientation.setRPY(1.64, 0.83, -3);
        target_pose.orientation = tf2::toMsg(orientation);

        target_pose.position.x = 0.0076;
        target_pose.position.y = 0.548;
        target_pose.position.z = 0.57;
        move_group_interface.setPoseTarget(target_pose);
        move_group_interface.move();
        ros::WallDuration(1.0).sleep();

        move_group_interface.rememberJointValues("pencil_pose");
        close_gripper();

        auto initial_joints = move_group_interface.getRememberedJointValues().at("initial");
        move_group_interface.setJointValueTarget(initial_joints);
        move_group_interface.move();
    }

    void draw_O() {
        std::vector<geometry_msgs::Pose> target_poses;
        target_poses.resize(5);

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, -0.36);

        target_poses[0].orientation = tf2::toMsg(orientation);
        target_poses[0].position.x = -0.3;
        target_poses[0].position.y = 0.45;
        target_poses[0].position.z = 0.5;

        target_poses[1].orientation = tf2::toMsg(orientation);
        target_poses[1].position.x = 0;
        target_poses[1].position.y = 0.45;
        target_poses[1].position.z = 0.7;

        target_poses[2].orientation = tf2::toMsg(orientation);
        target_poses[2].position.x = 0.3;
        target_poses[2].position.y = 0.45;
        target_poses[2].position.z = 0.5;

        target_poses[3].orientation = tf2::toMsg(orientation);
        target_poses[3].position.x = 0;
        target_poses[3].position.y = 0.45;
        target_poses[3].position.z = 0.3;

        target_poses[4].orientation = tf2::toMsg(orientation);
        target_poses[4].position.x = -0.3;
        target_poses[4].position.y = 0.45;
        target_poses[4].position.z = 0.5;

        for (const auto &target : target_poses) {
            move_group_interface.setPoseTarget(target);
            move_group_interface.move();
        }
    }

    void drop_pencil() {
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    panda.add_objects();
    panda.take_pencil();
//    panda.draw_O();
//    panda.drop_pencil();

    return 0;
}