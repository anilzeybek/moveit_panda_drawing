#include "letter_poses.h"
#include <ctype.h>

const float point_matrix[5][5][3] = {
    {{0.4, -0.4, 0.8}, {0.2, -0.4, 0.8}, {0, -0.4, 0.8}, {-0.2, -0.4, 0.8}, {-0.4, -0.4, 0.8}},
    {{0.4, -0.4, 0.7}, {0.2, -0.4, 0.7}, {0, -0.4, 0.7}, {-0.2, -0.4, 0.7}, {-0.4, -0.4, 0.7}},
    {{0.4, -0.4, 0.6}, {0.2, -0.4, 0.6}, {0, -0.4, 0.6}, {-0.2, -0.4, 0.6}, {-0.4, -0.4, 0.6}},
    {{0.4, -0.4, 0.5}, {0.2, -0.4, 0.5}, {0, -0.4, 0.5}, {-0.2, -0.4, 0.5}, {-0.4, -0.4, 0.5}},
    {{0.4, -0.4, 0.4}, {0.2, -0.4, 0.4}, {0, -0.4, 0.4}, {-0.2, -0.4, 0.4}, {-0.4, -0.4, 0.4}},
};

geometry_msgs::Point LetterPoses::get_position_at(int i1, int i2) {
    geometry_msgs::Point p;
    p.x = point_matrix[i1][i2][0];
    p.y = point_matrix[i1][i2][1];
    p.z = point_matrix[i1][i2][2];

    return p;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_A() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_B() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_C() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_D() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_E() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_F() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_G() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_H() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_I() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_J() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_K() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_L() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_M() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_N() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_O() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(9);

    tf2::Quaternion draw_orientation;
    draw_orientation.setRPY(M_PI, 0, -M_PI_4 * 3);
    for (auto &target : target_poses)
        target.orientation = tf2::toMsg(draw_orientation);

    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(1, 1);
    target_poses[2].position = get_position_at(0, 2);
    target_poses[3].position = get_position_at(1, 3);
    target_poses[4].position = get_position_at(2, 4);
    target_poses[5].position = get_position_at(3, 3);
    target_poses[6].position = get_position_at(4, 2);
    target_poses[7].position = get_position_at(3, 1);
    target_poses[8].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_P() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Q() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_R() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_S() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_T() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_U() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_V() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_W() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_X() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Y() {
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Z() {
}

std::vector<geometry_msgs::Pose> LetterPoses::get_poses(char letter) {
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