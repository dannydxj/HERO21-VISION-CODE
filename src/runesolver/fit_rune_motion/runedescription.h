#ifndef RUNE_DESCRIPTION_H
#define RUNE_DESCRIPTION_H

#include <iostream>
#include <vector>

#include "types.h"
#include "util_func.h"
#include "circlequeue.h"


struct LineSpeed
{
    double speed; // 线速度值
    int64_t clock; // 该线速度对应的时钟
};

struct Palstance
{
    double  palstance; // 速度值
    int64_t clock; // 该角速度对应的时钟
};

class RuneDescriptior{
public:
    /// 能量机关平面距机器在y轴上的距离
    constexpr static double STANDARD_LENGTH = 6.9;

    /// 能量机关标准半径,单位m
    constexpr static double STANDARD_RUNE_RADIUS = 0.70;


    /// 用于判断点的密集程度,单位m
    constexpr static double min_delta_dis = 0.0174533;

public:
    /// 匀速能量机关描述
    ConstRuneData const_rune_data;

    /// 变速能量机关描述
    PivRuneData piv_rune_data;

    /// 上一帧识别到的装甲板
    Target last_target;

    /// 当前帧识别到的待激活的装甲板
    Target ptz_target;

    /// 三维转换坐标后值，用于拟合圆，单位m
    CircleQueue<Target, 301> points;

    /// 用于记录一个扇叶的点，切换扇叶清空，重新积累
    CircleQueue<Target, 301> single_fan_points;

    /// 拟合实际圆心，能量机关平面内
    cv::Point2d c;

    /// 拟合半径,单位m
    double radius;

    /// 拟合圆成功的标志
    int flag_fit_circle_success = 0;

    /// 转动方向，0表示未知，1表示逆时针，-1表示顺时针
    int rotate_dir = 0;

    /// 线速度序列点
    CircleQueue<LineSpeed, 300> line_speed_set;

    /// 角速度滤波器, 单位度/s
    double S_palstance_filter = 0;
    double L_palstance_filter = 0;
    int filter_cnt = 0;

    /// 寻找到时间零点的标志位
    int flag_speed_clock = 0;

    /// 时间零点
    int64_t speed_clock = 0;

    /// 判断是否进行密集点除处理佛如标志位
    bool flag_handle_density;

    /// 输出信息
    std::fstream rune_info_file;

public:
    void init(const cv::FileStorage &file_storage);

    void loadParam(const cv::FileStorage &file_storage);

    void runSmallRune(ImageClass &image_object, const ReadPack &mcu_data, const std::vector<Candidate_Rect> &candidate_rects, int energy_yaw);

    void runBigRune(ImageClass &image_object, const ReadPack &mcu_data, const std::vector<Candidate_Rect> &candidate_rects, int energy_yaw);


    void onEnter();

    void onExit();

    void clear(); 

    /**
     * @brief 最小二乘法拟合圆
     */
    bool calibrate(CircleQueue<Target, 301> &m_Points, cv::Point2d &Centroid, double &dRadius);  //拟合圆

    /**
     * @brief 计算目标点，单位m
     */
    void RunesolvePnP4Points(const std::vector<Candidate_Rect> &todo_candidate_rects, std::vector<Target> &targets);

    /**
     * @brief 能量机关坐标系变换
     */
    void RunetransCoordinate(std::vector<Target> &targets, double pitch, double yaw, double roll, double energy_yaw);

    /**
     * @brief 判断扇叶是否切换
     * @details 判断新一帧扇叶与上一帧扇叶的空间距离，超过阈值即认为扇叶切换
     */
    bool judgeFansSwitched(const Target &curr_target, const Target &last_target);

    /**
     * @brief 判断转动方向
     */
    void judgeRotateDir();

    /**
     * @brief 从点集的循环队列中寻找时间跨度合适的点计算角速度
     */
    bool getPointToComputepalstance(Target &target);

    /**
     * @brief 计算角速度滤波器
     */
    void computePalstanceFliter();

    /**
     * @brief 求得两个相邻angle的角度之差
     * @param angle 单位度
     */
    double getDeltaAngle(double front_angle, double back_angle);

    /**
     * @brief 计算角速度, 逆时针为正.rad/s
     * @details 可能会存在较大的误差
     */
    double computePalstance(const Target target1, const Target target2);

    /**
     * @brief 获得时间零点
     */
    bool getSpeedClock();

    /**
     * @brief 获得两个坐标点的绝对空间距离 
     */
    double get3Ddistance(const Target &target1, const Target &target2);

    /**
     * @brief 将计算的特性打包近一个结构体，便于预测取值
     */
    void setConstRuneData(int clock);

    void setPivRuneData(int clock);



public:
        
    /// 能量机关装甲板实际尺寸，单位mm
    /// 半宽，始终为长边
    double half_w;
    /// 半高，始终为短边
    double half_h;

    /// 相机内参矩阵
    cv::Mat CAMERA_MATRIX;

    /// 相机畸变矩阵
    cv::Mat DISTORTION_COEFF;

    /// pnp解算的旋转矩阵
    cv::Mat rotate_mat;

    /// pnp解算的平移矩阵
    cv::Mat trans_mat;
};

#endif