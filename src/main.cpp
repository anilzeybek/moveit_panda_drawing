#include "PandaRobot.h"
#include "Environment.h"
#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_panda_drawing");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PandaRobot panda;
    Environment env;

    env.add_objects();
    panda.pick_pencil(env.get_pencil_location());
    panda.draw_letter('B');
    panda.place_pencil();

    return 0;
}