#include "PandaRobot.h"
#include "Environment.h"
#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "panda_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    Environment env;

    env.add_objects();
    panda.pick_pencil(env.get_pencil_location());
    panda.draw('Z');
    panda.place_pencil();

    return 0;
}