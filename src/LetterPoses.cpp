#include "LetterPoses.h"
#include <cctype>

std::vector<std::vector<std::array<float, 3>>> LetterPoses::point_matrix;
int LetterPoses::x_index = 0, LetterPoses::y_index = 0;

void LetterPoses::generate_point_matrix() {
    float interval = 0.025;

    float x_start = 0.3;
    float x_end = -0.3;

    float y_start = 0.8;
    float y_end = 0.3;

    float i = y_start;
    while (i >= y_end) {
        float j = x_start;
        std::vector<std::array<float, 3>> v;
        while (j >= x_end) {
            v.push_back({j, -0.4, i});
            j -= interval;
        }

        LetterPoses::point_matrix.push_back(v);
        i -= interval;
    }
}

geometry_msgs::Point LetterPoses::get_position_at(int i1, int i2) {
    geometry_msgs::Point p;
    int loc_x = i1 + LetterPoses::y_index;
    int loc_y = i2 + LetterPoses::x_index;

    p.x = point_matrix[loc_x][loc_y][0];
    p.y = point_matrix[loc_x][loc_y][1];
    p.z = point_matrix[loc_x][loc_y][2];

    return p;
}

void LetterPoses::set_target_orientation(std::vector<geometry_msgs::Pose> &poses) {
    tf2::Quaternion draw_orientation;
    draw_orientation.setRPY(M_PI, 0, -M_PI_4 * 3);

    for (auto &pose : poses)
        pose.orientation = tf2::toMsg(draw_orientation);
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_A() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(2, 1);
    target_poses[4].position = get_position_at(1, 1);
    target_poses[5].position = get_position_at(1, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_B() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(1, 0);
    target_poses[4].position = get_position_at(2, 1);
    target_poses[5].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_C() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(4);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 1);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(2, 0);
    target_poses[3].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_D() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(4);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(1, 1);
    target_poses[2].position = get_position_at(2, 0);
    target_poses[3].position = get_position_at(0, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_E() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(7);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 1);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(1, 0);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(1, 0);
    target_poses[5].position = get_position_at(2, 0);
    target_poses[6].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_F() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 1);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(1, 0);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(1, 0);
    target_poses[5].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_G() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 2);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(2, 0);
    target_poses[3].position = get_position_at(2, 2);
    target_poses[4].position = get_position_at(1, 2);
    target_poses[5].position = get_position_at(1, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_H() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);
    target_poses[2].position = get_position_at(1, 0);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(0, 1);
    target_poses[5].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_I() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(2);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_J() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(3);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 1);
    target_poses[1].position = get_position_at(2, 1);
    target_poses[2].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_K() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);
    target_poses[2].position = get_position_at(1, 0);
    target_poses[3].position = get_position_at(0, 1);
    target_poses[4].position = get_position_at(1, 0);
    target_poses[5].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_L() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(3);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);
    target_poses[2].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_M() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(1, 1);
    target_poses[3].position = get_position_at(0, 2);
    target_poses[4].position = get_position_at(2, 2);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_N() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(4);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_O() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(0, 1);
    target_poses[2].position = get_position_at(2, 1);
    target_poses[3].position = get_position_at(2, 0);
    target_poses[4].position = get_position_at(0, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_P() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(1, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Q() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(7);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(0, 1);
    target_poses[2].position = get_position_at(1, 1);
    target_poses[3].position = get_position_at(2, 2);
    target_poses[4].position = get_position_at(1, 1);
    target_poses[5].position = get_position_at(1, 0);
    target_poses[6].position = get_position_at(0, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_R() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(2, 0);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(1, 0);
    target_poses[4].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_S() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 1);
    target_poses[1].position = get_position_at(0, 0);
    target_poses[2].position = get_position_at(1, 0);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(2, 1);
    target_poses[5].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_T() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(4);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(0, 2);
    target_poses[2].position = get_position_at(0, 1);
    target_poses[3].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_U() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(4);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);
    target_poses[2].position = get_position_at(2, 1);
    target_poses[3].position = get_position_at(0, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_V() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(1, 0);
    target_poses[2].position = get_position_at(2, 1);
    target_poses[3].position = get_position_at(1, 2);
    target_poses[4].position = get_position_at(0, 2);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_W() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(2, 0);
    target_poses[2].position = get_position_at(1, 1);
    target_poses[3].position = get_position_at(2, 2);
    target_poses[4].position = get_position_at(0, 2);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_X() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(6);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(1, 1);
    target_poses[2].position = get_position_at(2, 2);
    target_poses[3].position = get_position_at(0, 0);
    target_poses[4].position = get_position_at(1, 1);
    target_poses[5].position = get_position_at(2, 0);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Y() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(1, 1);
    target_poses[2].position = get_position_at(0, 2);
    target_poses[3].position = get_position_at(1, 1);
    target_poses[4].position = get_position_at(2, 1);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::poses_Z() {
    std::vector<geometry_msgs::Pose> target_poses;
    target_poses.resize(5);

    set_target_orientation(target_poses);
    target_poses[0].position = get_position_at(0, 0);
    target_poses[1].position = get_position_at(0, 2);
    target_poses[2].position = get_position_at(1, 1);
    target_poses[3].position = get_position_at(2, 0);
    target_poses[4].position = get_position_at(2, 2);

    return target_poses;
}

std::vector<geometry_msgs::Pose> LetterPoses::get_poses(char letter) {
    letter = toupper(letter);

    switch (letter) {
    case 'A':
        return poses_A();
    case 'B':
        return poses_B();
    case 'C':
        return poses_C();
    case 'D':
        return poses_D();
    case 'E':
        return poses_E();
    case 'F':
        return poses_F();
    case 'G':
        return poses_G();
    case 'H':
        return poses_H();
    case 'I':
        return poses_I();
    case 'J':
        return poses_J();
    case 'K':
        return poses_K();
    case 'L':
        return poses_L();
    case 'M':
        return poses_M();
    case 'N':
        return poses_N();
    case 'O':
        return poses_O();
    case 'P':
        return poses_P();
    case 'Q':
        return poses_Q();
    case 'R':
        return poses_R();
    case 'S':
        return poses_S();
    case 'T':
        return poses_T();
    case 'U':
        return poses_U();
    case 'V':
        return poses_V();
    case 'W':
        return poses_W();
    case 'X':
        return poses_X();
    case 'Y':
        return poses_Y();
    case 'Z':
        return poses_Z();
    default:
        return *new std::vector<geometry_msgs::Pose>;
    }
}

void LetterPoses::increase_x_index(int val) {
    LetterPoses::x_index += val;
}

void LetterPoses::increase_y_index(int val) {
    LetterPoses::y_index += val;
}

void LetterPoses::reset_x_index() {
    LetterPoses::x_index = 0;
}

void LetterPoses::reset_y_index() {
    LetterPoses::y_index = 0;
}