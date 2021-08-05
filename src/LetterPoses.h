#ifndef LETTER_POSES_H
#define LETTER_POSES_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LetterPoses {
private:
    static void set_target_orientation(std::vector<geometry_msgs::Pose> &poses);

    static geometry_msgs::Point get_position_at(int i1, int i2);

    static std::vector<geometry_msgs::Pose> poses_A();

    static std::vector<geometry_msgs::Pose> poses_B();

    static std::vector<geometry_msgs::Pose> poses_C();

    static std::vector<geometry_msgs::Pose> poses_D();

    static std::vector<geometry_msgs::Pose> poses_E();

    static std::vector<geometry_msgs::Pose> poses_F();

    static std::vector<geometry_msgs::Pose> poses_G();

    static std::vector<geometry_msgs::Pose> poses_H();

    static std::vector<geometry_msgs::Pose> poses_I();

    static std::vector<geometry_msgs::Pose> poses_J();

    static std::vector<geometry_msgs::Pose> poses_K();

    static std::vector<geometry_msgs::Pose> poses_L();

    static std::vector<geometry_msgs::Pose> poses_M();

    static std::vector<geometry_msgs::Pose> poses_N();

    static std::vector<geometry_msgs::Pose> poses_O();

    static std::vector<geometry_msgs::Pose> poses_P();

    static std::vector<geometry_msgs::Pose> poses_Q();

    static std::vector<geometry_msgs::Pose> poses_R();

    static std::vector<geometry_msgs::Pose> poses_S();

    static std::vector<geometry_msgs::Pose> poses_T();

    static std::vector<geometry_msgs::Pose> poses_U();

    static std::vector<geometry_msgs::Pose> poses_V();

    static std::vector<geometry_msgs::Pose> poses_W();

    static std::vector<geometry_msgs::Pose> poses_X();

    static std::vector<geometry_msgs::Pose> poses_Y();

    static std::vector<geometry_msgs::Pose> poses_Z();

public:
    static std::vector<geometry_msgs::Pose> get_poses(char letter);
};

#endif