/**
 * @file targetsolver.h
 * @brief PnP解算
 * @details 得到相机坐标系
 */

#ifndef TARGETSOLVER_HPP
#define TARGETSOLVER_HPP
#include <fstream>
#include <opencv2/opencv.hpp>

#include "aimbot.h"
#include "armor.h"
/**
 * @brief 目标姿态解算类
 * 解算出目标装甲板在云台坐标系中的x,y,z坐标
 */
class TargetSolver {
   public:
    /// 大装甲板实际宽度的一半, 单位为毫米
    constexpr static double HALF_BIG_ARMOR_WIDTH = 112.5;

    /// 大装甲板实际高度的一半, 单位为毫米
    constexpr static double HALF_BIG_ARMOR_HEIGHT = 27.50;

    /// 小装甲板实际宽度的一半, 单位为毫米
    constexpr static double HALF_SMALL_ARMOR_WIDTH = 65.0;

    /// 小装甲板实际高度的一半, 单位为毫米
    constexpr static double HALF_SMALL_ARMOR_HEIGHT = 27.50;

    /// 设置解算模式，计算速度: P3P > EPNP > Iterative, 精确度反之
    constexpr static int ALGORITHM = cv::SOLVEPNP_ITERATIVE;

    /// 相机内参矩阵, 常量
    cv::Mat CAMERA_MATRIX;

    /// 相机畸变系数, 常量
    cv::Mat DISTORTION_COEFF;

    /// PNP解算结果旋转矩阵
    cv::Mat rotate_mat;

    /// PNP解算平移矩阵
    cv::Mat trans_mat;

    fstream ofs;

   public:
    /**
     * @brief 默认构造函数
     */
    TargetSolver();

    /**
     * @brief 默认析构函数
     */
    ~TargetSolver();

    /**
     * @brief 初始化函数
     *
     * @param file_storage 配置文件
     */
    void init(const cv::FileStorage &file_storage);

    /**
     * @brief 装甲板在相机坐标下解算
     * @param rect 目标装甲板旋转矩形
     * @param is_big_armor 是否是大装甲板, 大小装甲板拥有不同的实际尺寸
     */
    void armor_getCoord(const Armor &armor, Target &target);

    /**
     * @brief 相机坐标系转换成云台坐标系
     *
     * @param camera_position 输入相机坐标
     * @param ptz_position 输出云台坐标
     */
    void camera2ptz(const cv::Mat &camera_position, Target &ptz_position);

    /**
     * @brief 用于输出坐标
     */
    void exportCoord(const Target &target);
};

#endif  // TARGETSOLVER_HPP
