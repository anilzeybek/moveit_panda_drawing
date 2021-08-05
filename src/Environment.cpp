#include "Environment.h"

moveit_msgs::CollisionObject
Environment::add_object(const std::string &name, int type, const std::vector<double> &dimensions,
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

void Environment::add_objects() {
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

geometry_msgs::Pose Environment::get_pencil_location() {
    return planning_scene_interface.getObjectPoses({"pencil"}).at("pencil");
}
