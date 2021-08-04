#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>


class Environment {
private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    static moveit_msgs::CollisionObject
    add_object(const std::string &name, int type, const std::vector<double> &dimensions,
               const std::vector<double> &positions);

public:
    void add_objects();

    geometry_msgs::Pose get_pencil_location();
};


#endif
