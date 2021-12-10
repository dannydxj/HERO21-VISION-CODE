#ifndef RUNEDETECTOR_HPP
#define RUNEDETECTOR_HPP

#include <math.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>

#include "base.h"
#include "circlequeue.h"
#include "log.h"
#include "macro_switch.h"
#include "timer.h"
#include "types.h"
#include "util_func.h"

class RuneDetector {
public:
    /// 是否开始识别标志位
    int flag_start = 0;

    /// todo为待激活，理论上应该有且只有一个，done为已激活，理论上（0-5）
    /// 每次循环结束后都要clear!!!
    std::vector<Candidate_Rect> todo_candidate_rects, done_candidate_rects;

    /// 能量机关颜色
    int color;

    /// energy_yaw, 单位度
    double energy_yaw;

    /// 对心yaw
    double central_yaw;

    /// 对心pitch
    double central_pitch;

    /// 识别失败次数
    int fail_cnt = 0;

    /// 对心标志
    bool flag_center_aligning = false;

public:

    void init(const cv::FileStorage &file_storage);

    void loadParam(const cv::FileStorage &file_storage);

    bool run(ImageClass &image_object, const ReadPack &mcu_data);

    void onEnter();

    void onExit();

    /**
     * @brief 对图像做形态学处理
     */
    void processImage(cv::Mat &origin_image, const int clolor);

    /**
     * @brief 寻找目标
     */
    bool findTarget(cv::Mat &processed_image,
                    std::vector<Candidate_Rect> &todo_candidate_rects,
                    std::vector<Candidate_Rect> &done_candidate_rects);


    /**
     * @brief 二值化函数，使得阈值在一个区间内，适用于低曝光条件下，颜色比较明显
     */
    void threshImage(cv::Mat &image, int color);

    /**
     * @brief 在图上绘制装甲板和圆等
     */
    void drawToDebug();

    /**
     * @brief
     */
    void clear();


public:
    /// 高斯卷积核大小
    int gauss_kernel_size = 1;

    /// 膨胀操作卷积核大小
    int dilate_kernel_size = 1;

    /// 闭运算卷积核大小
    int close_kernel_size = 1;

    /// 开运算卷积核大小
    int open_kernel_size = 1;

    /// 膨胀操作卷积核
    cv::Mat dilate_kernel;

    /// 闭运算卷积核
    cv::Mat close_kernel;

    /// 开运算卷积核
    cv::Mat open_kernel;

    /// 灰度图二值化阈值
    int RUNE_GRAY_THRES;

    /// 蓝通道减红通道二值化阈值
    int RUNE_BLUE_THRESH;

    /// 红通道减蓝通道二值化阈值
    int RUNE_RED_THRESH;

    /// 蓝通道二值化阈值,同时也是最小的
    int RUNE_BLUE_CHANNEL_THRESH;

    /// 红通道二值化阈值，同时也是最小的
    int RUNE_RED_CHANNEL_THRESH;

    /// 子轮廓最小面积
    int RUNE_MIN_CHILD_CONTOUR_AREA;

    /// 子轮廓最大面积
    int RUNE_MAX_CHILD_CONTOUR_AREA;

    /// 父轮廓最小面积
    int RUNE_MIN_DAD_CONTOUR_AREA;

    /// 父轮廓最大面积
    int RUNE_MAX_DAD_CONTOUR_AREA;

    /// 子轮廓最小长度
    int RUNE_MIN_CHILD_CONTOUR_LENGTH;

    /// 子轮廓最大长度
    int RUNE_MAX_CHILD_CONTOUR_LENGTH;

    /// 父轮廓最小长度
    int RUNE_MIN_DAD_CONTOUR_LENGTH;

    /// 父轮廓最大长度
    int RUNE_MAX_DAD_CONTOUR_LENGTH;


    /// 原图像的浅拷贝，用于debug
    cv::Mat debug_image;

    cv::Mat src_image;

    /// 灰度图
    cv::Mat gray_image;

    /// 二值图
    cv::Mat binary_image;

    /// 通道相减图
    cv::Mat subtract_image;

    /// 开运算得到的图,暂未用到
    cv::Mat open_image;

    /// 漫水算法得到的图,暂未用到
    cv::Mat flood_image;

    /// 最终的图像
    cv::Mat processed_image;
};

#endif