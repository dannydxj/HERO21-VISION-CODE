/**
 * @file types.h
 * @brief 自定义类型集合
 * @details 存放各种自定义类型, 包括通信协议数据帧, 机器人工作模式等
 * @license 2021 HITWH HERO-Robomaster Group
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <opencv2/opencv.hpp>

#define ERROR 0


/**
 * @brief 枚举敌方装甲板颜色
 */
enum EnemyColor {
    /// 默认由电控控制
    COLOR_AUTO = 0,

    /// 调试时固定红色
    COLOR_RED = 1,

    /// 调式时固定蓝色
    COLOR_BLUE = 2,
};

/**
 * @brief 枚举模式
 * 比赛场上依据不同的需求所设定的模式
 */
enum Mode {
    /// 由电控控制
    MODE_AUTO = 0,

    /// 二代自瞄
    MODE_ARMOR2,

    /// 匀速能量机关
    MODE_SMALLRUNE,

    /// 变速能量机关
    MODE_BIGRUNE,

    /// 击打哨兵
    MODE_SENTINEL,

    /// 能量机关二次数据
    MODE_VICE_RUNE,

    /// 英雄吊射
    MODE_HERO

};

/**
 * @brief 电控发来的数字含义
 */
enum EnemyNumber {
    /// 使用数字识别，但不追踪数字
    USE_MODEL_NO_TRACK = 0,

    /// 打建筑，哨兵优先极最高
    SHOOT_BUILDING = 6,

    /// 电控端强制退出数字识别
    EXIT_MODEL = 7
};

/**
 * @brief 通信包类型
 */
enum CommunicationPackType {
    /// 从 MCU 接收的对时包
    READ_PACK_FOR_TIME = 0,
    /// 从 MCU 接收的姿态包
    READ_PACK_FOR_POSE
};

/**
 * @brief 云台相应坐标
 * 云台坐标系中装甲板中心的三个坐标
 */
typedef struct Target {
    /// x轴坐标，单位mm
    double x;

    /// y轴坐标，单位mm
    double y;

    /// z轴坐标，单位mm
    double z;

    /// 时钟时间,单位ms
    int64_t clock;

    Target() : x(0.0), y(0.0), z(0.0), clock(0) {}

    /**
     * @brief 重载流输出运算符
     *
     * @param out 标准输出流
     * @param target Target 结构体
     * @return 标准输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const Target &target) {
        out << "target: "
            << "[" << target.x << ", " << target.y << ", " << target.z << "]";
        return out;
    }
} Target;

/**
 * @brief 发送数据包
 * 发送与电控通讯间的相应所需数据
 */
struct SendPack {
    /// 自瞄参数
    /// 工作模式
    int mode;

    /// 预测pitch，单位度
    double pred_pitch;

    /// 预测yaw，单位度
    double pred_yaw;

    /// yaw精度,半角宽，单位度
    double yaw_resolution;

    /// pitch精度,半角宽，单位度
    double pitch_resolution;

    /// 预测yaw轴转速,单位度/s
    double yaw_palstance;

    /// 预测pitch轴转速,单位度/s
    double pitch_palstance;

    /// 运动预测是否给出目标
    bool target_found;

    /// 是否有把握开火
    bool fire;

    /// 设定值时刻
    int64_t clock;


    /// 当前能量机关装甲板角位置,适用于小能量机关和静止能量机关
    double small_rune_angle;

    /// 延迟时间
    double time_delay;

    ///爆发式自瞄,默认没被触发
    bool burstAim;

    /**
     * @brief 构造函数，初始化成员变量
     */
    SendPack()
        : mode(Mode::MODE_ARMOR2),
          pred_yaw(0.0),
          pred_pitch(0.0),
          time_delay(0.0),
          burstAim(false) {}

    /**
     * @brief 重载流输出运算符
     *
     * @param out 标准输出流
     * @param send_pack SendPack 结构体
     * @return 标准输出流
     */
    friend std::ostream &operator<<(std::ostream &out,
                                    const SendPack &send_pack) {
        // x, y, z 已经在 Target 里面打印了，就不需要在这里打印了
        out << "work_mode: " << send_pack.mode << std::endl
            << "pred_yaw: " << send_pack.pred_yaw << std::endl
            << "pred_pitch: " << send_pack.pred_pitch << std::endl
            << "time_delay: " << send_pack.time_delay << std::endl
            << "burstAim: " << send_pack.burstAim << std::endl;
        return out;
    }
};

/**
 * @brief 接收数据包
 * 从电控接收到视觉这边所需要的信息
 */
struct ReadPack {
    /// 包的类型
    int pack_type;

    /// 对时包的电控基准时刻（单位：ms）
    int64_t base_mcu_time;

    /// 姿态包的电控时刻（未补全，最原始的情况）（单位：ms）
    int64_t pose_mcu_time;

    /// 自瞄模式
    int mode;

    /// 当前云台yaw偏角
    double ptz_yaw;

    /// 当前云台pitch偏角
    double ptz_pitch;

    /// 当前云台roll偏角
    double ptz_roll;

    /// 当前子弹速度
    double bullet_speed;

    /// 敌方颜色
    int enemy_color;

    /// 要追踪的数字 默认为0，代表不进行追踪
    int enemy_number;

    /// 是否按下了鼠标右键(脉冲沿)
    bool isRightMouseButtonClicked;

    /// 接受到的鼠标状态
    bool isRightMouseButtonPressing;

    /// 能量机关深度标准距离
    double standard_length;

    /// 能量机关 yaw
    double energy_yaw;

    /**
     * @brief 构造函数，初始化成员变量
     */
    ReadPack()
        : pack_type(CommunicationPackType::READ_PACK_FOR_TIME),
          base_mcu_time(0),
          pose_mcu_time(0),
          mode(Mode::MODE_ARMOR2),
          ptz_yaw(0.0),
          ptz_pitch(0.0),
          ptz_roll(0.0),
          bullet_speed(0.0),  // TODO: 需要有一个默认值
          enemy_color(EnemyColor::COLOR_RED),
          enemy_number(0),
          isRightMouseButtonClicked(false),
          isRightMouseButtonPressing(false),
          standard_length(6.8),
          energy_yaw(0.0) {}

    /**
     * @brief 重载流输出运算符
     *
     * @param out 标准输出流
     * @param read_pack ReadPack 结构体
     * @return 标准输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const ReadPack &read_pack) {
        // `mode` 已经在 SendPack 中打印了，这里无需打印
        // isvalid 不需要打印，因为并不是接受的信息
        out << "pack_type: " << read_pack.pack_type << std::endl
            << "base_mcu_time: " << read_pack.base_mcu_time << std::endl
            << "pose_mcu_time: " << read_pack.pose_mcu_time << std::endl
            << "ptz_pitch: " << read_pack.ptz_pitch << std::endl
            << "ptz_yaw: " << read_pack.ptz_yaw << std::endl
            << "ptz_roll: " << read_pack.ptz_roll << std::endl
            << "bullet_speed: " << read_pack.bullet_speed << std::endl
            << "own_color: " << read_pack.enemy_color << std::endl
            << "enemy_number: " << read_pack.enemy_number << std::endl
            << "isRightMouseButtonClicked: " << read_pack.isRightMouseButtonClicked << std::endl
            << "standard_length: " << read_pack.standard_length << std::endl
            << "energy_yaw: " << read_pack.energy_yaw << std::endl; 
            return out;
    }
};



/**
 * @brief 能量机关候选矩形，存储旋转矩形和它与标准面积的差值
 * @brief rect 装甲板最小外接矩形
 * @param isActivated 是否被激活
 */
struct Candidate_Rect {
    cv::RotatedRect rect;
    bool isActivated;
    Candidate_Rect() : isActivated(false) {}
};

/**
 * @brief 灯条类型
 * @param rate 轮廓面积与包围的矩形面积之比
 * @param rect minAreaReact拟合矩形、长宽值准确
 * @param elli_rect fitEllipse拟合矩形，角度值准确
 * @param angle 灯条角度
 * @param color 灯条颜色
 */
class Lightbar {
   public:
    cv::RotatedRect rect;
    cv::RotatedRect elli_rect;
    double rate;
    double angle;
    int color;

   public:
    static bool coordinateComparator(const Lightbar &a, const Lightbar &b) {
        if (a.rect.center.x == b.rect.center.x) {
            return (a.rect.center.y < b.rect.center.y);
        } else {
            return (a.rect.center.x < b.rect.center.x);
        }
    }
};

/**
 * @brief 陀螺数据
 * @param radius 半径
 * @param center 圆心
 * @param palstance 角速度
 */
struct GyroData {
    double radius;
    cv::Point2d center;
    double palstance;
};

/**
 * @brief 自瞄预测直线
 * @param m_n(一维拟合直线描述)
 * @param m(x:x坐标；y:y坐标；z:z坐标)
 * @param n(slope:斜率，intercept:截距, index:相关系数)
 * @param clock 拟合直线的最新点对应的时刻
 * @param status 该直线描述的运动状态
 * @details 速度单位m/s (mm/ms)
 */
struct Line {
    double x_slope;
    double x_intercept;
    double x_index;
    double y_slope;
    double y_intercept;
    double y_index;
    double z_slope;
    double z_intercept;
    double z_index;
    int64_t clock;
    int status;
    Target target;
    bool valid;
    Line() : x_slope(0.0), x_intercept(0.0), x_index(0.0),
             y_slope(0.0), y_intercept(0.0), y_index(0.0),
             z_slope(0.0), z_intercept(0.0), z_index(0.0),
             status(0), clock(0), valid(false){}

};


/**
 * @brief 控制云台的数据
 * @param palstance 角速度 单位：度/s
 */
struct GimbalData{
    double pitch;
    double yaw;
    double pitch_palstance;
    double yaw_palstance;
};

// TODO: 姿态结构体
struct Pose {
    double ptz_pitch;
    double ptz_yaw;
    double ptz_roll;
    /// 姿态时刻（已补全）
    int64_t mcu_time;
};

/**
 * @brief 图片类，记录图片id(0-255循环)
 */
struct ImageClass {
    cv::Mat source_image;
    Pose pose;
};

/**
 * @brief 能量机关描述
 */
 struct ConstRuneData{
    double x;
    double z;
    double Dir;
    double radius;
    Target target; // 最近的一张图片拟合的装甲板
    int64_t clock; // 最近的一张图片对应的时刻
 };

struct PivRuneData
{
    double x;
    double z;
    double Dir;
    double radius;
    Target target; // 最近的一张图片拟合的装甲板
    int64_t clock; // 最近的一张图片对应的时刻
    int64_t speed_clock; // 速度零点标志位 
};

#endif  // TYPES_H
