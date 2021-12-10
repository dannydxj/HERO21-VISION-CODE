/**
 * @file aimbot.h
 * @brief 装甲板检测类头文件
 * @details 从图像中找出装甲板
 * @license Copyright© 2020 HITwh HERO-RoboMaster Group
 */

#ifndef AIMBOT_H
#define AIMBOT_H

#include <algorithm>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

#include "armor.h"
#include "base.h"
#include "circlequeue.h"
#include "classifier_dk.h"
#include "classifier_sj.h"
#include "log.h"
#include "macro_switch.h"
#include "types.h"

#ifdef COMPILE_WITH_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#define TOP_NUM 30

/**
 * @brief 自瞄状态
 */
enum AimState {
    /// 搜索装甲板状态
    SEARCH_STATE = 1,

    /// 追踪装甲板状态
    TRACK_STATE = 2
};

/**
 * @brief 装甲板检测类
 */
class AimBot {
public:
    /// 陀螺速度阈值,超过该值之后自动进入反小陀螺模式
    constexpr static double GYRO_LOW_SPEED_THRESH = 0.5;

    /// 
    constexpr static double GYRO_HIGH_SPEED_THRESH = 1.5;

public:
    /// 自瞄当前模式，在开始时执行赋值
    int curr_state;

    /// 下一帧自瞄模式,在结束时执行赋值
    int next_state;

    /// 记录丢失帧数
    int lost_cnt = 0;

    /// 追踪状态下最大允许丢失帧数
    // TODO 最大允许丢失帧数需要测试
    int max_lost_cnt = 2;

    /// 搜寻成功标志
    bool targetSearched;

    /// 当前帧追踪成功
    bool trackSuccess;

    /// 追踪正常进行中（最大丢帧数小于max_lost_cnt）
    bool targetTracked = false;

    /// 当前追踪的数字，仅在搜索状态下更新！
    int curr_number;

    /// 判断接下来一帧是否进行数字识别
    /// 该值只取决于距离，且同时与USE_MODEL为真时才真正启用数字识别
    bool model_work_status = true;

    ///
    bool anti_gyro_status;

    /// 是否使用数字识别
    int USE_MODEL;
    
    /// 是否使用ROI
    int ROI_ENABLE;

    /// 装甲板相对于相机绝对位置，单位m
    double absoluteDistance = 0.0;

    /// 候选灯条
    std::vector<Lightbar> lightbars;

    /// 候选装甲板
    std::vector<Armor> armors;

    /// 当前是否找到装甲板
    bool armorFound;

    /// 当前识别到的待打击装甲板
    Armor target_armor;

    /// 上一帧识别的装甲板
    Armor old_armor;

    /// ROI区域
    cv::Rect roi_rect;

    /// 历史坐标点
    CircleQueue<Target, TOP_NUM> his_targets;

    /// 陀螺数据
    GyroData gyro_data;

    /// 
    bool track_status;

    /// 切换装甲板
    bool flag_switch_armor;

   public:
    /**
     * @brief 默认构造函数
     */
    AimBot();

    /**
     * @brief 默认析构函数
     */
    ~AimBot();

    /**
     * @brief 初始化函数
     *
     * @param file_storage 参数配置文件
     */
    void init(const cv::FileStorage &file_storage);

    /**
     * @brief 执行装甲板检测的核心函数
     *
     * @param src 源图像
     * @param mcu_data 通信包
     * @return 是否找到装甲板
     *  @retval true 找到装甲板
     *  @retval false 没有找到装甲板
     */
    bool run(cv::Mat &image, const ReadPack &mcu_data, int dir, double speed);

    /**
     * @brief 循环进入自瞄模式的一些设置
     * @details 比如根据上一帧的状态设置当前帧的状态等
     */
    void onEnter();

    /**
     * @brief 循环退出自瞄的一些操作
     * @details 比如清空一些数据容器
     */
    void onExit();

    /**
     * @brief 寻找装甲板模式，全图搜索
     */
    bool searchArmor(cv::Mat &image, const ReadPack &mcu_data);

    /**
     * @brief 追踪装甲板模式，roi区域或者是全图检索
     */
    bool trackArmor(cv::Mat &image, const ReadPack &mcu_data, int dir, double speed);

    /**
     * @brief 判断对面是否在进行陀螺运动
     * @details 只有在对方陀螺速度超过一定值后才开启陀螺模式
     * @details 陀螺打击策略：yaw轴指向拟合圆心， pitch轴跟随当前装甲板更新
     */
    bool judgeGyro(const CircleQueue<Target, TOP_NUM> &his_targets,
                   GyroData &gyro_data);

    /**
     * @brief 加载参数
     */
    void loadParam(const cv::FileStorage &file_storage);

    /**
     * @brief 根据对方装甲板颜色, 将图像预处理成二值图像
     *
     * @param src 源图像
     * @param mcu_color 敌方颜色
     */
    void processImage(const cv::Mat &src, const int mcu_color);

    /**
     * @brief 寻找可能灯条
     */
    void findLightbars(const cv::Mat &processed_image, const int mcu_color);

    /**
     * @brief 灯条匹配装甲板
     */
    void matchArmors(const std::vector<Lightbar> &lightbars,
                     std::vector<Armor> &armors);

    /**
     * @brief 装甲板打分
     */
    void gradeArmors(std::vector<Armor> &armors);

    /**
     * @brief 搜索模式下数字识别过滤和排序
     */
    bool trySearchArmor(std::vector<Armor> &armors, int mcu_number,
                        int mcu_color);

    /**
     * @brief 利用位置追踪装甲板，在数字识别开启的情况下同时用数字识别过滤
     * @details 注意里面有反陀螺操作
     * @param speed 描述陀螺运动速度
     * @param dir 运动方向(左:-1, 右:1)
     */
    bool tryTrackArmor(std::vector<Armor> &armors, int mcu_number, int dir, double speed);

    /**
     * @brief 反慢速小陀螺

     */
    void antiLowSpeedGyro(double speed, int dir);

    /**
     * @brief 根据装甲板那的误差分数和优先级进行排序
     */
    void sortArmors(std::vector<Armor> &armors);

    /**
     * @brief 判断是否进行爆发式自瞄打击
     * @details 满足条件后在发送的数据包所含数据id+128
     * @details 条件详见函数定义
     */
    void judgeBurstAim(std::vector<Armor> &armors, int mcu_number);

    /**
     * @brief 根据距离来决定是否使用数字识别
     * @details
     * 因为近距离会存在曝光过高光晕严重影响数字的情况，所以距离小于1.5m不使用数字识别
     * @details
     */
    void ApplyModelBaseOnDistance();

    /**
     * @brief 判断装甲板是否切换
     */
    bool judgesArmorSwitched(const Armor &curr_armor, const Armor &old_armor);

    /**
     * @brief 防止roi区域超出图像区域（640 * 480）
     */
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

    /**
     * @brief 把一些识别信息放到显示到图片上
     * @details 信息例如：自瞄模式/是否使用数字识别/是否爆发式自瞄
     */
    void displayInfo(cv::Mat &debug_image);

    /**
     * @brief 用于从其他模式转换到自瞄模式重新运行一些设置,例如工作状态的初始化
     */
    void config();

    const cv::Rect &getRoiRect() const;

    void setRoiRect(const cv::Rect &roiRect);

   public:
    /// 原图像的浅拷贝，用于debug
    cv::Mat debug_image;

    /// 预处理得到的灰度图像, 用于寻找灯条
    cv::Mat processed_image;

    /// ROI图片
    cv::Mat roi_image;

    /// 灰度图
    cv::Mat gray_image;

    /// 二值图
    cv::Mat binary_image;

    /// 通道相减图
    cv::Mat subtract_image;

    /// 颜色通道二值图
    cv::Mat channel_thresh_image;

    /// 轮廓被视为灯条的最大小面积
    double MAX_LIGHTBAR_AREA;
    double MIN_LIGHTBAR_AREA;

    /// 灯条长短比下限
    double MIN_LIGHTBAR_RATIO;

    /// 大装甲板长短比下限
    double MIN_ASPECT_BIG_RATIO;

    /// 大装甲板长短比上限
    double MAX_ASPECT_BIG_RATIO;

    /// 小装甲板长短比下限
    double MIN_ASPECT_SMALL_RATIO;

    /// 小装甲板长短比上限
    double MAX_ASPECT_SMALL_RATIO;

    /// 两灯条长度比上限
    double MAX_LENGTH_RATIO;

    /// 灯条倾斜度上限
    double MAX_LIGHTBAR_ANGLE;

    /// 灯条倾斜度之差上限
    double MAX_LIGHTBAR_DELTA;

    /// 装甲板倾斜度上限
    double MAX_ARMOR_ANGLE;

    /// 每帧送进armor.cpp和分类器的装甲板的最大数量
    int MAX_CANDIDATE_NUM;

    /// 源图像二值化阈值
    int BLUE_BINARY_THRES;
    int RED_BINARY_THRES;

    /// 颜色通道阈值
    int BLUE_CHANNEL_THRESH;
    int RED_CHANNEL_THRESH;

    /// 通道相减图像灰度阈值,用于第一种图像预处理方法
    int BLUE_SUBTRACT_THRES;
    int RED_SUBTRACT_THRES;

    /// 算子核大小
    int KERNEL_SIZE;

    /// 相机内参矩阵, 常量
    cv::Mat CAMERA_MATRIX;

    /// 相机的fy，由内参矩阵得出
    double camera_fy;

#ifdef DISTORTION_CORRECT
    Mat camera_matrix;
    Mat distortion_coeff;
#endif  // DISTORTION_CORRECT

#ifndef COMPILE_WITH_CUDA
    cv::Mat kernel;
#else
    cv::Ptr<cv::cuda::Filter> kernel;
#endif  // COMPILE_WITH_CUDA

    /// 上交数字分类器
    Classifier_sj classifier_sj;

    /// darknet分类器
    Classifier_dk classifier_dk;
};

#endif  /// AIMBOT_H
