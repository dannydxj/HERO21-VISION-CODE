/**
 * @file workspace.hpp
 * @brief 工作类头文件
 * @details 机器人核心工作类, 多线程工作, 完成图像接收, 图像处理, 通信功能
 * @license Copyright© 2021 HITwh HERO-RoboMaster Group
 */

#ifndef WORKSPACE_HPP
#define WORKSPACE_HPP

#include <mutex>
#include <chrono>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <exception>
#include <sys/time.h>
#include <mutex>
#include <string.h>

#include "armor.h"
#include "aimbot.h"
#include "runedetector.h"
#include "hit_sentinel.h"
#include "targetsolver.h"
#include "debugger.h"
#include "camera.h"
#include "mvcamera.h"
#include "cannode.h"
#include "log.h"
#include "timer.h"
#include "util_func.h"
#include "armor_predict.h"
#include "runedetector.h"
#include "runedescription.h"
#include "runepredictor.h"
#include "circlequeue.h"
#include "sj_queue.h"
#include "can_to_debug.h"


/// 配置文件路径<br>
/// 开自启时需改为绝对路径
const static std::string PARAM_PATH = "../Config/param.xml";

/**
 * @brief 工作类
 * 完成多线程, 图像接收, 图像处理, 通信功能
 */
class Workspace
{
public:
    /// 线程锁
    std::mutex image_buffer_mutex;

    /// 姿态队列锁
    std::mutex pose_queue_metux;

    /// 自瞄预测锁
    std::mutex pred_mutex;

    /// 通信包赋值锁
    std::mutex msg_mutex;

    /// 用于录像
    cv::VideoWriter writer;

    /// 装甲板检测类对象
    AimBot armor_detector;
    
    /// 坐标解算类对象
    TargetSolver target_solver;
    
    /// 自瞄预测类
    ArmorPredictor armor_predictor;

    /// 能量机关识别类
    RuneDetector rune_detector;

    /// 能量机关运动描述类
    RuneDescriptior rune_descriptior;

    /// 能量机关预测类
    RunePredictor rune_predictor;

    /// 击打哨兵类
    Sentinel sentinel_hunter;

    /// 当前被处理的图像对象
    ImageClass curr_image_object;

    /// 相机对象
    MVCamera *camera = new MVCamera(this);

    /// 视频流对象
    cv::VideoCapture cap;

    CanNode can_node;
#ifdef USE_CAN_TO_DEBUG
    DebugCAN debug_can;
#endif

    /// 相机图像缓冲区
    std::vector<ImageClass> image_object_buffer;

    /// 当前被处理的图像
    cv::Mat image_original;

    ///目标装甲板
    Armor target_armor;

    // 计数器（记录图片id)
    int img_id_cnt;

    // 保存视频计数器
    // int video_cnt = 0;

    /// 目标三维坐标 
    Target target;

    /// 用来做自瞄运动预测的数据点
    CircleQueue<Target, ARMOR_PRED_LNUM> his_coords;

    /// 向 MCU 发送的数据包
    SendPack send_pack;

    /// 最初从 cannode 中接收数据的包
    ReadPack original_msg;

    /// 工作包
    ReadPack work_msg;

    /// 姿态队列（只会在通信接收线程中被修改，图像接收线程中读取，但由于 Pose 是结构体，多于 64 位，因此读操作也需要加锁）
    CircleQueue<Pose, 10> pose_queue;

    /// 用于在 workspace 中获取视觉时刻和电控时刻的计时器
    // 视觉时刻取相对于该计时器启动时
    Timer workspace_timer;

    /// 是否显示图像
    int SHOW_IMAGE = 0;

    /// 是否显示滚动条<br>
    /// 必须在 SHOW_IMAGE=1 的情况下才可开启！
    int TRACKBAR = 0;

    /// 是否显示运行时间
    int RUNNING_TIME = 0;


    /// 敌方设定颜色：0代表从电控读；1是红色，2是蓝色
    int ENEMY_COLOR;

    /// 机器人设定工作模式
    int MODE;

    /// 是否使用相机，是1否0
    /// ***注意：不使用相机时默认为使用视频传入***
    int USE_CAMERA = 1;

    /// 是否使用CAN通信，1使用CAN, 0不使用CAN
    int USE_CAN = 0;

    /// 是否在**运行代码的同时**保存视频，0否1是
    int SAVE_VIDEO = 1;

    /// 当前帧图片运算时间, 子弹发射延迟中要考虑进去
    float delay_time; 

    /// 
    int video_now = 0;

    ///
    int video_past;

    /// 测试帧总帧数
    int totalFrame = 0;

    /// 当前帧数占总帧数百分比
    int curr_frame_ratio = 0;

    /// 测试视频输入路径
    std::string VIDEO_PATH;

    /// 视频保存路径
    std::string VIDEO_SAVE_PATH;

    /// CAN通信flag， 只有在开启电控通信后我方才开始向电控发送信息
    int can_flag = 0;

    /// 运动拟合点集标志位,为防止访问冲突
    int armor_flag = 0;

    /// 记录每次发送信息对应的mcu时钟
    int clockAtsend = 0;

    /// 触发模式下，时间帧对齐参数
    int64_t soft_triger_synchronizor = 8;

    /// 连续取图模式下，时间帧对齐参数
    int64_t constant_synchronizor = 8;


    /// 记录保存视频的帧数
    int video_cnt = 0;

public:
    /**
     * @brief 默认构造函数
     */
    Workspace();

    /**
     * @brief 默认析构函数
     */
    ~Workspace();

    /**
     * @brief 初始化
     */
    void init(const cv::FileStorage &file_storage);

    /**
     * @brief 导入参数
     */
    void loadParam(const cv::FileStorage &file_storage);

    /**
     * @brief 检查工作模式是否冲突
     */
    void checkParamConflicts();

    /**
     * @brief 通信初始化
     */
    void initCommunication();

    /**
     * @brief 开启多线程
     */
    void run();

    void initVideoRecorder();

    void recordVideo(cv::Mat &image);

public:
    /**
     * @brief 图像接收线程, 可以从图片和视频文件中接收, 也可以从工业相机中接收
     */
    void imageReceivingFunc();

    /**
     * @brief 图像处理线程, 完成对目标的检测, 跟踪, 预测, 角度解算, 向MCU发送数据包
     */
    void imageProcessingFunc();

    /**
     * @brief 通信接收线程, 读取MCU发送的数据包
     */
    void readCommunicatingFunc();

    /**
     * @brief 通信发送线程，发送预测的设定值
     */
    void sendCommunicatingFunc();

    /**
     * @brief 处理通信的信息
     */
    void handleMessage(ReadPack &readpack);

    /**
     * @brief 从缓存区中获取图片，在使用相机的时候调用
     */
    bool getImgFromBuffer();

    /**
     * @brief 设置工作模式
     * 
     */
    void setMode();

    /**
     * @brief 设置颜色
     */
    void setColor(); 
    
    /**
     * @brief 设置相机的工作模式
     */
    void setCamera(int last_mode, int curr_mode);

    /**
     * @brief 调试时显示图像 & 创建滚动条
     */
    void showImage();

    /**
     * @brief 补偿量
     */
    void compensateSkew(ReadPack &read_pack);

    /**
     * @brief 自瞄模式图像处理逻辑函数
     *
     */
    void armorFunc();

    /**
     * @brief 能量机关模式图像处理逻辑函数
     *
     */
    void RuneFunc();

    /**
     * @brief 击打模式图像处理逻辑函数
     */
    void hitSentinelFunc();
    
    /**
     * @brief 发送自瞄预测数据
     */
    void setArmorPredictdata(SendPack &send_pack, int64_t clock_now);

    /**
     * @brief 发送自瞄预测数据
     */
    void setSentinelPredictdata(SendPack &send_pack, int64_t clock_now);

    /**
     * @brief 发送小能量机关预测数据
     */
    void setSmallRunePredData(SendPack &send_pack, int64_t clock_now);

    /**
     * @brief 发送大能量机关预测数据
     */
    void setBigRunePredData(SendPack &send_pack, int64_t clock_now);

    /**
     * @brief 读取的视频
     */
    void resizeVideo(cv::Mat &image);

    /**
     * @brief 为了方便分析视频
     * @details 默认顺序播放， s键保存并暂停，空格键暂停但不保存， q键退出
     */
    void VideoAnalyse(cv::Mat image);

    /**
     * @brief 判断是否应该唤醒沉睡的发送线程,控制发送线程每10ms运行一次
     */
    bool judgeIfApproSendWindowPhase();
};

#endif /// WORKSPACE_HPP
