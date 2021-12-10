/**
 * @file armor.h
 * @brief 装甲板类头文件
 * @details 定义存储装甲板对象各种信息的装甲板类
 * @license Copyright© 2020 HITwh HERO-RoboMaster Group
 */
#ifndef ARMOR_H
#define ARMOR_H

#include <opencv2/opencv.hpp>

#include "base.h"
#include "types.h"
#include "util_func.h"

/**
 * @brief 装甲板类
 * 用于存储装甲板对象的各种参数并对其进行操作，部分属性不进行封装,
 * 降低getter、setter带来的开销
 */
class Armor {
   public:
    /// 装甲板打击优先级, 优先级越高越优先击打 TODO
    int priority;

    /// 装甲板数字编号的图像
    cv::Mat number_img;

    /// 输入的roi图像
    cv::Mat src;

    /// 输入的矩形
    cv::Rect roi_rect;

    ///自适应曝光灰度图
    cv::Mat exposure_gray;

    /// 贴合装甲板区域的旋转矩形
    cv::RotatedRect armor_rect;
    cv::RotatedRect lbar_rect;
    cv::RotatedRect rbar_rect;

    /// 左右灯条中心点
    cv::Point2f left_center, right_center;

    /// 敌方装甲板颜色
    int mcu_color;

    /// 用于爆发式自瞄的判断
    bool burstAim = false;

    /// 是否为大装甲板
    bool isBigArmor = false;

    /// 分类器推理的装甲板数字编号
    int classifier_num;

    /// 总误差分数
    double sum_error_score = 0;

    /// 距离误差分数
    double distance;
    double dis_error_score;

    /// 装甲板误差分数
    double ratio;
    double ratio_error_score;

    /// 两灯条高度比例误差分数
    double bar_len_ratio;
    double bar_len_ratio_error_score;

    /// 两灯条倾斜度之差误差分数
    double bar_delta_angle;
    double bar_delta_angle_error_score;

    /// 装甲板的灯条与矩形的相似程度造成的误差分数
    double similarity;
    double similarity_error_score;

    /// 装甲板中心到图像中心的x轴距离误差分数，未置用
    double x_axis_bias;
    double x_axis_bias_error_score;

    // 装甲板倾斜程度误差分数
    double armor_angle;
    double armor_angle_error_score;

    /// 装甲板高度
    double height;

    /// 装甲板宽度
    double width;

    /// 装甲板与上一帧装甲板的偏移距离
    double offset_distance = 0;

   private:
    /// 旋转矩形宽度放大倍率, 尽可能保证截取数字区域的全部特征
    constexpr static double ROTATEDRECT_WIDTH_RATE = 1.0;

    /// 旋转矩形高度放大倍率, 尽可能保证截取数字区域的全部特征
    constexpr static double ROTATEDRECT_HEIGHT_RATE = 2.0;

   public:
    /**
     * @brief 默认构造函数
     */
    Armor();

    /**
     * @brief 默认析构函数
     */
    ~Armor();

    /**
     * @brief 根据左右灯条给armor类成员赋值
     */
    void assignMemberVariables(const Lightbar &lbar, const Lightbar &rbar);

    cv::Rect rect();

    /**
     * @brief 设置数字图像
     */
    void setNumberImg(int mcu_number);

    /**
     * @brief 获取装甲板数字编号
     *
     * @return 装甲板数字编号(常量）
     */
    int getNumber();

    /**
     * @brief
     */
    static bool setOffsetDistance(const Armor &a, const Armor &b);

    /**
     * @brief 设置装甲板数字编号, 同时更新优先级
     *
     * @param number 装甲板数字编号
     */
    void setPriority(int classifier_num, int mcu_num);

    /**
     * @brief 自定义排序函数, 按误差得分升序排列装甲板对象
     *
     * @param a 装甲板对象1（在该函数中不可修改）
     * @param b 装甲板对象2（在该函数中不可修改）
     * @return 返回表示排序结果的布尔值
     *   @retval true 表示a排在前面
     *   @retval false 表示b排在前面
     */
    static bool scoreComparator(const Armor &a, const Armor &b);

    /**
     * @brief 自定义排序函数, 按打击优先级得分降序排列装甲板对象
     *
     * @param a 装甲板对象1（在该函数中不可修改）
     * @param b 装甲板对象2（在该函数中不可修改）
     * @return 返回表示排序结果的布尔值
     *   @retval true 表示a排在前面
     *   @retval false 表示b排在前面
     */
    static bool priorityComparator(const Armor &a, const Armor &b);

    /**
     * @brief 防止截取的数字区域超出图像区域
     */
    void preventExceed(int &x, int &y, int &width, int &height,
                       const cv::Mat &src);


   /**
    * @brief 将装甲板那的一些信息打印到终端上 
    */
   void listInfo();
};

#endif  // ARMOR_H