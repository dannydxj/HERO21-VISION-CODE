#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>
#include <string>
#include <iostream>

#include "circlequeue.h"

/**
 * @brief 计时器类
 * 用于获取打印各模块的时间，得到算法效率信息
 * 注意，该函数只能用在函数内部！
 */
class Timer {
   private:
    /// 计时器启动时的日历时间，Linux下为从1970年1月1日0时整到此时的时间，用结构体timeval存储
    timeval time_start;

    /// 计时器工作状态, true表示正在工作, false表示未工作
    bool is_open = false;

    /// 时间差队列（只会在通信接收线程中被修改和被读取）
    static CircleQueue<int64_t, 10> time_gap_queue;

    /// 时间差滤波结果队列（只会在通信接收线程中被修改，图像接收线程只是读取）
    static CircleQueue<int64_t, 10> time_gap_filter_queue;

   public:
    /**
     * @brief 默认构造函数
     */
    Timer();

    /**
     * @brief 默认析构函数
     */
    ~Timer();

    /**
     * @brief 打印计时器工作时间, 计时器必须处于工作状态
     *
     * @param message 需要打印运行时间的程序段描述信息
     */
    void printTime(const std::string &message) const;

    /**
     * @brief 获取计时器工作时间, 计时器必须处于工作状态
     * @return 工作时间, 单位为毫秒
     */
    double getTime() const;

    /**
     * @brief 获取时间差，进行滤波，并存入时间差滤波结果队列。
     * 计时器必须处于工作状态
     * 可以选择返回
     * @return 返回最新计算出的时间差滤波结果，单位 ms
     */
    int64_t getTimeGap(int64_t &base_mcu_time);

    /**
     * @brief 获取当前的电控时刻
     * @return 当前电控时刻, 单位为毫秒
     */
    int64_t getMCUTime();

    /**
     * @brief 重启计时器
     */
    void restart();
};

#endif  // TIMER_HPP