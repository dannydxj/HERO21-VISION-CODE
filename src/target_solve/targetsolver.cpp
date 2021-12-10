#include "targetsolver.h"

#include "timer.h"

TargetSolver::TargetSolver() = default;

TargetSolver::~TargetSolver() = default;

/**
 * @brief PnP解算初始化，引入内参矩阵和畸变矩阵
 */
void TargetSolver::init(const cv::FileStorage &file_storage) {
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    file_storage["distortion_coeff"] >> DISTORTION_COEFF;
}

// 空间坐标系（右手系）转换为相机坐标系下
void TargetSolver::armor_getCoord(const Armor &armor, Target &target) {
    cv::Point2f left_up, left_down, right_up, right_down;
    cv::Point2f left_vertices[4];
    cv::Point2f right_vertices[4];
    armor.lbar_rect.points(left_vertices);
    armor.rbar_rect.points(right_vertices);

    /* 按照y坐标进行排序。注意：y轴正方向向下
       将四个点的y坐标由小到大排序 */
    sort(left_vertices, left_vertices + 4,
         [](const cv::Point2f &p1, const cv::Point2f &p2) {
             return p1.y < p2.y;
         });
    // 再根据四个点的y坐标确定相关位置
    left_up = (left_vertices[0] + left_vertices[1]) * 0.5;

    left_down = (left_vertices[2] + left_vertices[3]) * 0.5;

    sort(right_vertices, right_vertices + 4,
         [](const cv::Point2f &p1, const cv::Point2f &p2) {
             return p1.y < p2.y;
         });

    right_up = (right_vertices[0] + right_vertices[1]) * 0.5;

    right_down = (right_vertices[2] + right_vertices[3]) * 0.5;

    double half_w, half_h;
    if (armor.isBigArmor) {
        half_w = HALF_BIG_ARMOR_WIDTH;
        half_h = HALF_BIG_ARMOR_HEIGHT;
    } else {
        half_w = HALF_SMALL_ARMOR_WIDTH;
        half_h = HALF_SMALL_ARMOR_HEIGHT;
    }

    /* 根据装甲板类型的不同，
       将装甲板中心作为空间坐标系的原点，
       其四个顶点作为2D-3D的四对点 */
    // points3d中的点需和points2d中的点按顺序一一对应
    static vector<cv::Point3f> points3d;
    points3d.clear();
    points3d.emplace_back(cv::Point3f(-half_w, -half_h, 0));
    points3d.emplace_back(cv::Point3f(half_w, -half_h, 0));
    points3d.emplace_back(cv::Point3f(half_w, half_h, 0));
    points3d.emplace_back(cv::Point3f(-half_w, half_h, 0));

    std::vector<cv::Point2f> points2d;
    points2d.clear();
    points2d.emplace_back(left_up);
    points2d.emplace_back(right_up);
    points2d.emplace_back(right_down);
    points2d.emplace_back(left_down);

    solvePnP(points3d, points2d, CAMERA_MATRIX, DISTORTION_COEFF, rotate_mat,
             trans_mat, false, ALGORITHM);
    // 暂时不需要使用旋转矩阵，需要用取消注释
    // Rodrigues(rotate_mat, rotate_mat);
    camera2ptz(trans_mat, target);
}

// 以米作单位，offset补偿值源于机器人摄像头与云台所设定的原点间有物理距离
void TargetSolver::camera2ptz(const cv::Mat &camera_position, Target &ptz_position) {
    ptz_position.x = ((camera_position.at<double>(0, 0) + Util::X_OFFSET)) / 1000.0;
    ptz_position.y = ((camera_position.at<double>(1, 0) + Util::Y_OFFSET)) / 1000.0;
    ptz_position.z = ((camera_position.at<double>(2, 0) + Util::Z_OFFSET)) / 1000.0;
}

void TargetSolver::exportCoord(const Target &target) {
    ofs.open("../param/test.csv", ios::out | ios::app);
    ofs << "," << target.x << "," << target.y << "," << target.z << endl;
    ofs.close();
}
