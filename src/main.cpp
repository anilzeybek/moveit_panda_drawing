#include <ros/ros.h>

// MoveIt libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PandaRobot {
   private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group;

    void open_gripper(trajectory_msgs::JointTrajectory &posture) {
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.04;
        posture.points[0].positions[1] = 0.04;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

    void close_gripper(trajectory_msgs::JointTrajectory &posture) {
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0;
        posture.points[0].positions[1] = 0;
        posture.points[0].time_from_start = ros::Duration(0.5);
    }

    moveit_msgs::CollisionObject add_object(const std::string &name, const std::vector<double> &dimensions, const std::vector<double> &positions) {
        moveit_msgs::CollisionObject object;

        object.id = name;
        object.header.frame_id = "panda_link0";

        object.primitives.resize(1);
        object.primitives[0].type = object.primitives[0].BOX;
        object.primitives[0].dimensions.resize(3);
        object.primitives[0].dimensions[0] = dimensions[0];
        object.primitives[0].dimensions[1] = dimensions[1];
        object.primitives[0].dimensions[2] = dimensions[2];

        object.primitive_poses.resize(1);
        object.primitive_poses[0].position.x = positions[0];
        object.primitive_poses[0].position.y = positions[1];
        object.primitive_poses[0].position.z = positions[2];

        object.operation = object.ADD;

        return object;
    }

   public:
    PandaRobot() : move_group("panda_arm") {
        ROS_INFO_STREAM("End-effector link: " << move_group.getEndEffectorLink());

        ros::WallDuration(1.0).sleep();
        move_group.setPlanningTime(45.0);
    }

    void add_objects() {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

        std::vector<double> dimensions{0.2, 0.4, 0.4};
        std::vector<double> positions{0.5, 0, 0.2};
        collision_objects.push_back(add_object("table", dimensions, positions));

        dimensions = {0.02, 0.02, 0.2};
        positions = {0.5, 0, 0.5};
        collision_objects.push_back(add_object("pencil", dimensions, positions));

        dimensions = {1.5, 0.05, 1};
        positions = {0, 0.6, 0.5};
        collision_objects.push_back(add_object("board", dimensions, positions));

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    void take_pencil() {
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.header.frame_id = "panda_link0";

        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI_2, -M_PI_4, -M_PI_2);

        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.x = 0.415;
        grasps[0].grasp_pose.pose.position.y = 0;
        grasps[0].grasp_pose.pose.position.z = 0.5;

        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.095;
        grasps[0].pre_grasp_approach.desired_distance = 0.115;

        grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
        grasps[0].post_grasp_retreat.direction.vector.x = -1.0;
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.1;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;

        open_gripper(grasps[0].grasp_posture);
        close_gripper(grasps[0].grasp_posture);

        move_group.setSupportSurfaceName("table");
        move_group.pick("pencil", grasps);
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
            move_group.setPoseTarget(target);
            move_group.move();
        }
    }

    void drop_pencil() {
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);
        place_location[0].place_pose.header.frame_id = "panda_link0";
        place_location[0].place_pose.pose.position.x = 0.5;
        place_location[0].place_pose.pose.position.y = 0;
        place_location[0].place_pose.pose.position.z = 0.5;

        place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
        place_location[0].pre_place_approach.direction.vector.x = 1;
        place_location[0].pre_place_approach.min_distance = 0.05;
        place_location[0].pre_place_approach.desired_distance = 0.1;

        open_gripper(place_location[0].post_place_posture);
        move_group.setSupportSurfaceName("table");
        move_group.place("pencil", place_location);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    panda.add_objects();
    panda.take_pencil();
    panda.draw_O();
    panda.drop_pencil();

    return 0;
}