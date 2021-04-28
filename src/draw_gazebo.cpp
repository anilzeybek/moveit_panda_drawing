#include <ros/ros.h>

// MoveIt libs
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PandaRobot {
private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group_interface;

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

    static void open_gripper(trajectory_msgs::JointTrajectory &posture) {
        posture.joint_names.resize(2);
        posture.joint_names[0] = "panda_finger_joint1";
        posture.joint_names[1] = "panda_finger_joint2";

        posture.points.resize(1);
        posture.points[0].positions.resize(2);
        posture.points[0].positions[0] = 0.04;
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
        ros::WallDuration(1.0).sleep();
        move_group_interface.setPlanningTime(45.0);

        // We can print the name of the reference frame for this robot.
        ROS_INFO_NAMED("draw", "Planning frame: %s",
                       move_group_interface.getPlanningFrame().c_str());

        // We can also print the name of the end-effector link for this group.
        ROS_INFO_NAMED("draw", "End effector link: %s",
                       move_group_interface.getEndEffectorLink().c_str());

        // We can get a list of all the groups in the robot:
        ROS_INFO_NAMED("draw", "Available Planning Groups:");
        std::copy(move_group_interface.getJointModelGroupNames().begin(),
                  move_group_interface.getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, ", "));
    }

    void add_objects() {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

        std::vector<double> dimensions{1, 1, 1};
        std::vector<double> positions{0, 1, 0};
        collision_objects.push_back(add_object("table", 0, dimensions, positions));

        dimensions = {0.10, 0.005};
        positions = {0, 0.65, 0.55};
        collision_objects.push_back(add_object("pencil", 1, dimensions, positions));

        dimensions = {1, 1, 1};
        positions = {0, -1, 0.5};
        collision_objects.push_back(add_object("board", 0, dimensions, positions));

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    void test() {
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = 0.28;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.5;

        move_group_interface.setPoseTarget(target_pose1);
        move_group_interface.move();
    }

    void take_pencil() {
        std::vector <moveit_msgs::Grasp> grasps;
        grasps.resize(1);
        grasps[0].grasp_pose.header.frame_id = "panda_link0";

        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI_2, -M_PI_4, -M_PI_2);

        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position.x = 0;
        grasps[0].grasp_pose.pose.position.y = 0.575;
        grasps[0].grasp_pose.pose.position.z = 0.5;

        grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
        grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.095;
        grasps[0].pre_grasp_approach.desired_distance = 0.115;

        grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
        grasps[0].post_grasp_retreat.direction.vector.y = -1.0;
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.1;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;

        open_gripper(grasps[0].grasp_posture);
        close_gripper(grasps[0].grasp_posture);

        move_group_interface.setSupportSurfaceName("table");
        move_group_interface.pick("pencil", grasps);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_draw");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
//     panda.test();
    panda.add_objects();
    panda.take_pencil();
    return 0;
}