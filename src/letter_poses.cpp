#include "letter_poses.h"
#include <ctype.h>

std::vector<geometry_msgs::Pose> poses_A() {
}

std::vector<geometry_msgs::Pose> poses_B() {
}

std::vector<geometry_msgs::Pose> poses_C() {
}

std::vector<geometry_msgs::Pose> poses_D() {
}

std::vector<geometry_msgs::Pose> poses_E() {
}

std::vector<geometry_msgs::Pose> poses_F() {
}

std::vector<geometry_msgs::Pose> poses_G() {
}

std::vector<geometry_msgs::Pose> poses_H() {
}

std::vector<geometry_msgs::Pose> poses_I() {
}

std::vector<geometry_msgs::Pose> poses_J() {
}

std::vector<geometry_msgs::Pose> poses_K() {
}

std::vector<geometry_msgs::Pose> poses_L() {
}

std::vector<geometry_msgs::Pose> poses_M() {
}

std::vector<geometry_msgs::Pose> poses_N() {
}

std::vector<geometry_msgs::Pose> poses_O() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(9);

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI_4 * 3);

    target_poses[0].orientation = tf2::toMsg(orientation);
    target_poses[0].position.x = 0.4;
    target_poses[0].position.y = -0.4;
    target_poses[0].position.z = 0.6;

    target_poses[1].orientation = tf2::toMsg(orientation);
    target_poses[1].position.x = 0.2;
    target_poses[1].position.y = -0.4;
    target_poses[1].position.z = 0.7;

    target_poses[2].orientation = tf2::toMsg(orientation);
    target_poses[2].position.x = 0;
    target_poses[2].position.y = -0.4;
    target_poses[2].position.z = 0.8;

    target_poses[3].orientation = tf2::toMsg(orientation);
    target_poses[3].position.x = -0.2;
    target_poses[3].position.y = -0.4;
    target_poses[3].position.z = 0.7;

    target_poses[4].orientation = tf2::toMsg(orientation);
    target_poses[4].position.x = -0.4;
    target_poses[4].position.y = -0.4;
    target_poses[4].position.z = 0.6;

    target_poses[5].orientation = tf2::toMsg(orientation);
    target_poses[5].position.x = -0.2;
    target_poses[5].position.y = -0.4;
    target_poses[5].position.z = 0.5;

    target_poses[6].orientation = tf2::toMsg(orientation);
    target_poses[6].position.x = 0;
    target_poses[6].position.y = -0.4;
    target_poses[6].position.z = 0.4;

    target_poses[7].orientation = tf2::toMsg(orientation);
    target_poses[7].position.x = 0.2;
    target_poses[7].position.y = -0.4;
    target_poses[7].position.z = 0.5;

    target_poses[8].orientation = tf2::toMsg(orientation);
    target_poses[8].position.x = 0.4;
    target_poses[8].position.y = -0.4;
    target_poses[8].position.z = 0.6;

    return target_poses;
}

std::vector<geometry_msgs::Pose> poses_P() {
}

std::vector<geometry_msgs::Pose> poses_Q() {
}

std::vector<geometry_msgs::Pose> poses_R() {
}

std::vector<geometry_msgs::Pose> poses_S() {
}

std::vector<geometry_msgs::Pose> poses_T() {
}

std::vector<geometry_msgs::Pose> poses_U() {
}

std::vector<geometry_msgs::Pose> poses_V() {
}

std::vector<geometry_msgs::Pose> poses_W() {
}

std::vector<geometry_msgs::Pose> poses_X() {
}

std::vector<geometry_msgs::Pose> poses_Y() {
}

std::vector<geometry_msgs::Pose> poses_Z() {
}

std::vector<geometry_msgs::Pose> get_poses(char letter) {
    letter = toupper(letter);

    switch (letter) {
    case 'A':
        return poses_A();
        break;
    case 'B':
        return poses_B();
        break;
    case 'C':
        return poses_C();
        break;
    case 'D':
        return poses_D();
        break;
    case 'E':
        return poses_E();
        break;
    case 'F':
        return poses_F();
        break;
    case 'G':
        return poses_G();
        break;
    case 'H':
        return poses_H();
        break;
    case 'I':
        return poses_I();
        break;
    case 'J':
        return poses_J();
        break;
    case 'K':
        return poses_K();
        break;
    case 'L':
        return poses_L();
        break;
    case 'M':
        return poses_M();
        break;
    case 'N':
        return poses_N();
        break;
    case 'O':
        return poses_O();
        break;
    case 'P':
        return poses_P();
        break;
    case 'Q':
        return poses_Q();
        break;
    case 'R':
        return poses_R();
        break;
    case 'S':
        return poses_S();
        break;
    case 'T':
        return poses_T();
        break;
    case 'U':
        return poses_U();
        break;
    case 'V':
        return poses_V();
        break;
    case 'W':
        return poses_W();
        break;
    case 'X':
        return poses_X();
        break;
    case 'Y':
        return poses_Y();
        break;
    case 'Z':
        return poses_Z();
        break;
    default:
        break;
    }
}