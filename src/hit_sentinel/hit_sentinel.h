#ifndef HITSENTINEL_HPP
#define HITSENTINEL_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "armor.h"
#include "targetsolver.h"
#include "types.h"

class Sentinel {
public:
    double k = 1.0;

    /// 浅拷贝，用于debug
    cv::Mat debug_image;

    /// 原图
    cv::Mat src;

   /// 是否使用roi
   int ROI_ENABLE;

    /// roi图像
    cv::Mat roi_image;

   /// ROI区域
    cv::Rect roi_rect;

    /// 灰度图
    cv::Mat gray_image;

    /// 二值图
    cv::Mat binary_image;

    cv::Mat subtract_image;


    /// 最终用作图像处理的图
    cv::Mat processed_image;

    /// 灯条集合
    std::vector<Lightbar> lightbars;

    /// 装甲板集合
    std::vector<Armor> armors;

    /// 是否找到目标
    bool target_found;

    /// 当前候选装甲板三维坐标
    std::vector<Target> targets;

    /// 待打击装甲板三维坐标
    Target ptz_target;

    /// 相机内参矩阵
    cv::Mat CAMERA_MATRIX;

    /// 相机畸变矩阵
    cv::Mat DISTORTION_COEFF;

    /// pnp解算的旋转矩阵
    cv::Mat rotate_mat;

    /// pnp解算的平移矩阵
    cv::Mat trans_mat;

   public:
    bool run(ImageClass &image_object, const int color);

    void onEnter();

    void onExit();

    /**
     * @brief 初始化
     */
    void init(const cv::FileStorage &file_storage);

    void loadParam(const cv::FileStorage &file_storage);

    /**
     * @brief 图像预处理
     */
    void processImg(cv::Mat &image, const int color);

    /**
     * @brief 寻找灯条
     */
    void findLightbars(const int color);

    /**
     * @brief 根据限定条件匹配装甲板
     */
    void matchArmors(std::vector<Lightbar> &lightbars,
                     std::vector<Armor> &armors);

    /**
     * @brief 针对出现多个装甲板的情况，根据其高度相对
     */
    void sortTarget(std::vector<Target> &targets);

    /**
     * @brief 选择待打击的装甲板
     */
    bool selectTarget(std::vector<Target> &targets);

    /**
     * @brief
     * 解算三维坐标，pnp解算后平移至yaw和pitch转轴中心，再转换到大地坐标系
     */
    void solve3Dcoordinates(std::vector<Armor> &armors, const ImageClass &image_object );

    /**
     * @brief 用深度距离和高度距离过滤装甲板
     */
    void filterTarget(std::vector<Target> &targets);

    /**
     * @brief 判读装甲板深度是否满足条件
     */
    bool ifDepthSatisfied(double armor_distance);

    /**
     * @brief 判读装甲板高度是否满足条件
     */
    bool ifHeightSatisfied(double armor_height);

    /**
     * @brief 根据高度进行坐标矫正
     */
    void rectifyCoordinate(std::vector<Target> &targets,
                           double standard_height);

    void setRoiRect(const cv::Rect &roiRect);

    void preventROIExceed(int &x, int &y, int &width, int &height);

    /**
     * @brief 在图像上画出识别灯条
     */
    void drawBars(cv::Mat &debug_image, std::vector<Lightbar> &bars);

    /**
     * @brief 在图像上画出装甲板
     * @details 待击打装甲板绿色，其余候选装甲板红色,
     * old_armor（要追踪的装甲板）白色
     */
    void drawArmors(cv::Mat &debug_image, std::vector<Armor> &armors);




public:
    /// 标准高度
    double STANDARD_HEIGTH;

    /// 最低高度，单位m
    double MIN_HEIGHT;

    /// 最高高度，单位m
    double MAX_HEIGHT;

    /// 最近深度，单位m
    double MIN_DEPTH;

    /// 最远深度，单位m
    double MAX_DEPTH;

    /// 装甲板最小比例
    double MIN_ARMOR_RATIO;

    /// 装甲板最大比例
    double MAX_ARMOR_RATIO;

    /// 最小装甲板面积
    int MIN_ARMOR_AREA;

    /// 最大装甲板面积
    int MAX_ARMOR_AREA;

    /// 灯条高度比例最大值
    double MAX_BAR_HEIGHT_RATIO;

    /// 灯条最大角度差
    int MAX_BAR_DELTA_ANGLE;

    /// 灰度二值化阈值
    /// 红通道
    int RED_GRAY2BINARY_THRESH;
    int BLUE_GRAY2BINARY_THRESH;

    /// 颜色通道相减二值化阈值
    int RED_SUBTRACT_THRESH;
    int BLUE_SUBTRACT_THRESH;


    /// 单颜色通道二值化阈值
    int COLOR_CHANNEL_THRESH;
};

#endif  // HITSENTINEL_HPP