/**
 * @file mvcamera.h
 * @brief MindVision相机驱动类头文件
 * @details
 实现对工业摄像头的基本参数设定和初始化，并将当前帧的数据转化为opencv的Mat类型.
            具体可参考MindVision的官方linux SDK，下附SDK下载链接
 * @license Copyright© 2021 HITwh HERO-RoboMaster Group
 */

#ifndef MVCAMERA_H
#define MVCAMERA_H

#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <mutex>

#include "CameraApi.h"
#include "base.h"
#include "camera.h"
#include "log.h"
#include "macro_switch.h"
#include "timer.h"
#include "types.h"
#include "sj_queue.h"

// 用于两个头文件相互包含的解决办法
class Workspace;

/**
 * @brief MindVision相机驱动类
 * 实现对工业摄像头的基本参数设定和初始化，并将当前帧的数据转化为opencv的Mat类型
 */
class MVCamera {
public:
    /// 循环队列取图
    Queue<ImageClass, 2> image_object_buffer;

public:
    /// 相机句柄
    int hCamera;

    /// 软触发获取的图片
    ImageClass image_object;

    /// workspace 对象的指针
    Workspace *p_workspace;

    /// 软触发图片有效标志位
    bool soft_trigger_image_valid = false;

    /// 帧宽
    int frame_width = 640;

    /// 帧高
    int frame_height = 480;

    /// 曝光时间, 主要是用来输出
    double exposure_time;

    /// 帧率,<br>0: low <br> 1: normal <br> 2: high
    int current_frame_speed_index = 2;

    /// B通道增益，默认100
    int bchannel_gain = 100;

    /// G通道增益，默认100
    int gchannel_gain = 100;

    /// R通道增益，默认100
    int rchannel_gain = 100;

    /// 饱和度，默认100
    int saturation = 100;

    /// 对比度，默认100
    int contrast = 100;

    /// 伽马值，默认100
    int gamma = 100;

    /// 图像降噪开关
    int enableNoiseFilter = 1;

    /// 设置图像锐化程度[0-100],默认0
    int sharpness = 0;

    /*******设置手动曝光相关参数************************/
    /// 自瞄曝光时间，需要动态调节,注意是double这个数据类型
    double armor_exposure_time = 3000;

    /// 能量机关曝光时间
    double rune_exposure_time = 1500;

    double sentinel_exposure_time = 1000;

    /// 模拟增益
    int analog_gain = 64;
    /*****************END************************************/

    /****** 设置自动曝光相关参数**********************/
    /// 设置自动曝光目标亮度值, int
    int auto_target_brightness = 50;

    /// 设置自动曝光阈值, int
    int auto_thresh = 5;

    /// 设置自动曝光 曝光时间最小值, double
    double auto_min_expourse_time = 100;

    /// 设置自动曝光 曝光时间最大值, double
    double auto_max_expourse_time = 6000;

    /// 模拟增益默认设置为最高
    /// 设置自动曝光的模拟增益最小值, int
    int auto_min_analog_gain = 256;

    /// 设置自动曝光的模拟增益最大值, int
    int auto_max_analog_gain = 256;
    /*************END*************************/

    /// 处理后图像输出的缓冲区
    unsigned char *g_pRgbBuffer;

   private:

    /// 工业摄像头设备列表数组
    tSdkCameraDevInfo tCameraEnumList;

    /// 相机特性描述的结构体
    tSdkCameraCapbility tCapability;

    /// 图像的帧头信息
    tSdkFrameHead sFrameInfo;

    /// 指向图像的数据的缓冲区
    unsigned char *pbyBuffer;

    /// 相机工作状态
    bool is_open;

    /// 相机信息记录文件
    std::fstream camera_info_file;

   public:
    /**
     * @brief 构造函数
     */
    MVCamera(Workspace *p_workspace_);

    /**
     * @brief 析构函数，关闭相机
     */
    ~MVCamera();

    /**
     * @brief 主要是加载一些参数
     */
    void init(const cv::FileStorage &file_storage);

    /**
     * @brief 打开相机, MV-SUA33GC-T的最大分辨率为640*480
     */
    void open();

    /**
     * @brief 返回相机是否已开启
     *
     * @return bool值，表示相机是否开启
     *     @retval true 相机正常开启
     *     @retval false 相机未开启
     */
    bool isOpen();

    /**
     * @brief 通过相机获取单张Mat类型图片
     * @overload
     *
     * @param image的引用，保存获取到的图像
     */
    void getImage(cv::Mat &image);

    /**
     * @brief 计算拍照的电控时刻
     * 
     * @param workspace_timer 拍照时间计时器
     * @param synchronizor 时间帧对齐参数
     * @return 计算得出的拍照时的电控时刻
     */
    int64_t getShutterMCUTime(Timer &workspace_timer, int64_t synchronizor);

    /**
     * @brief 软触发函数
     * 发出软触发信号，并且将图片有效置为 false
     * 等到该次触发接收到图像之后，才能将图片有效置为 true
     */
    bool soft_trigger();

    /**
     * @brief 获取相机的各项信息，写入一个 txt 文本文件中
     */
    void getCameraInfo();

    /**
     * @brief 自瞄模式下相机设置
     */
    void CameraSetForAromr();

    /**
     * @brief 打击哨兵模式下相机设置
     */
    void CameraSetForSentinel();

    /**
     * @brief 能量机关模式下相机设置
     */
    void CameraSetForRune();

    /**
     * @brief 关闭相机
     */
    void close();
};

#endif  // MVCAMERA_HPP