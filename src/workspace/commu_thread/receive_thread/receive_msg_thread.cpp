#include "workspace.h"

/// 姿态时间补全函数
static int64_t fillPoseMCUTime(int64_t pose_mcu_time_unfilled, Timer &workspace_timer);

void Workspace::readCommunicatingFunc() 
{
    if (!USE_CAN) 
    {
        return;
    }

    while (true) 
    {
        Timer commu_timer;
        can_node.receive(original_msg);
        msg_mutex.lock(); // TODO 
        int last_mode = work_msg.mode;
        compensateSkew(original_msg);
        handleMessage(original_msg); // 纠正时钟
        msg_mutex.unlock();
        if (RUNNING_TIME) 
        {
            commu_timer.printTime("通信频率");
        }
    }
}

void Workspace::handleMessage(ReadPack &read_pack) 
{
    Pose temp_pose;
    can_flag = 1;

    if (read_pack.pack_type == CommunicationPackType::READ_PACK_FOR_TIME)
    {
        // 先获取时间差，进行滤波，并存入 workspace_timer 的时间差队列
        workspace_timer.getTimeGap(read_pack.base_mcu_time);
    } else if (read_pack.pack_type == CommunicationPackType::READ_PACK_FOR_POSE)
    { 
        temp_pose.ptz_pitch = read_pack.ptz_pitch;
        temp_pose.ptz_yaw = read_pack.ptz_yaw;
        temp_pose.ptz_roll = read_pack.ptz_roll;
        temp_pose.mcu_time = fillPoseMCUTime(read_pack.pose_mcu_time, workspace_timer);
        // 对于姿态队列，存入要加锁
        pose_queue_metux.lock();
        pose_queue.push(temp_pose);
        pose_queue_metux.unlock();

    } else 
    {
        LOGW("error pack type!");
    }
}
static int64_t fillPoseMCUTime(int64_t pose_mcu_time_unfilled, Timer &workspace_timer) 
{
    // 当前计算得出的电控时刻
    int64_t current_mcu_time = workspace_timer.getMCUTime();
    // 当前整数秒 + 电控发送的零头
    int64_t temp = (current_mcu_time / 1000) * 1000 + pose_mcu_time_unfilled;
    // 检查是否差了整秒
    int64_t delta_time = temp - current_mcu_time;
    // 最终补全的时间
    int64_t full_pose_mcu_time;
    if (delta_time > 500) {
        full_pose_mcu_time = temp - 1000;
    } else if (delta_time < -500) {
        full_pose_mcu_time = temp + 1000;
    } else {
        full_pose_mcu_time = temp;
    }
    return full_pose_mcu_time;
}
