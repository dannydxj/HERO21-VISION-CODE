// 图像处理线程
// 1. 发送对时信息
// 2. 从相机缓存或者视频中获取图片到缓存冲区中,带有异常检查，并可以限制帧率
// 3. 录制视频

#include "workspace.h"

static void linear_predict_pose(Pose& result, Pose& pose1, Pose& pose2);

void Workspace::imageReceivingFunc() 
{
    if (!USE_CAMERA)
    {
        return;
    } 

#ifdef SOFT_TRIGGER_MODE
    /// 记录丢失帧数的文件
    fstream file_lost_frame;
    file_lost_frame.open("../logs/lost_frame.txt", ios::out);
    /// 读图计时器，记录整个读图过程用了多长时间
    Timer image_getting_timer;
    /// 读图开始计时时刻
    double image_getting_start_time;
    /// 拍照时间计时器，用于计算拍照时的电控时间
    Timer shutter_timer;
    /// 记录丢失总帧数
    int count_total_lost_frame = 0;
    while (true) 
    { 
        // 开始读图
        if (RUNNING_TIME) 
        {
            image_getting_start_time = image_getting_timer.getTime(); // 类似于得到开始值
        }
        // 发送一次触发拍照信号
        if (!camera->soft_trigger())
        {
            continue;
        }
        // 计算拍照时的电控时刻(触发信号时刻加上曝光时间的一半)
        camera->image_object.pose.mcu_time = camera->getShutterMCUTime(workspace_timer, soft_triger_synchronizor);
        Timer allower;
        while (allower.getTime() < 20) //最多允许20ms
        {
            if(camera->soft_trigger_image_valid) // 接受到图像
            {
                // allower.printTime("get image time");
                break;
            }
        }
        if (!camera->soft_trigger_image_valid)
        {
            LOGE("get image error!");
            count_total_lost_frame++;
            file_lost_frame << "tolal lost frames: " << count_total_lost_frame << std::endl;
        } else // 获取到图片
        {
            // 图片获取到之后，需要线性预测图片拍摄时的云台姿态
            // 获取最新的两个云台姿态
            if (pose_queue.length > 2) // 初始
            {
                pose_queue_metux.lock();
                Pose pose1;
                pose_queue.back(pose1, 2);
                Pose pose2;
                pose_queue.back(pose2, 1);
                pose_queue_metux.unlock();
                if (abs(pose2.mcu_time - workspace_timer.getMCUTime()) < 100) // 保证pitch和yaw的实时性
                {
                    linear_predict_pose(camera->image_object.pose, pose1, pose2);
                    // 存入图像缓存区
                    image_buffer_mutex.lock();
                    image_object_buffer.emplace_back(camera->image_object);
                    image_buffer_mutex.unlock();
                }
            } else 
            {
                LOGE("couldn''t get pose data， thus can't get image");
            }
        }
        if (SAVE_VIDEO == 1 && !camera->image_object.source_image.empty()) 
        {
            recordVideo(camera->image_object.source_image);
        }
        if (RUNNING_TIME) 
        {
            cout  << "读图所用时间："
                  << image_getting_timer.getTime() - image_getting_start_time
                  << "ms" << endl;
        }
    }
#endif
}


// 线性姿态预测函数
void linear_predict_pose(Pose& result, Pose& pose1, Pose& pose2) 
{
    // 以 pose1.mcu_time 为时间原点
    // 计算各姿态分量的增量
    double delta_ptz_pitch = pose2.ptz_pitch - pose1.ptz_pitch;
    double delta_ptz_yaw = pose2.ptz_yaw - pose1.ptz_yaw;
    double delta_ptz_roll = pose2.ptz_roll - pose1.ptz_roll;
    // 计算 ptz-t 直线的斜率
    int64_t delta_t = (pose2.mcu_time - pose1.mcu_time);
    double k_ptz_pitch = 1.0 * delta_ptz_pitch / delta_t;
    double k_ptz_yaw = 1.0 * delta_ptz_yaw / delta_t;
    double k_ptz_roll = 1.0 * delta_ptz_roll / delta_t;   
    // 得到直线即为 result.ptz = k * result.mcu_time + pose1.ptz

    result.ptz_pitch = k_ptz_pitch * (result.mcu_time - pose1.mcu_time) + pose1.ptz_pitch;
    result.ptz_yaw = k_ptz_yaw * (result.mcu_time - pose1.mcu_time) + pose1.ptz_yaw;
    result.ptz_roll = k_ptz_roll * (result.mcu_time - pose1.mcu_time) + pose1.ptz_roll;
}