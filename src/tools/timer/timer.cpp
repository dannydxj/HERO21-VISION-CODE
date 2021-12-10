#include "timer.h"

//TODO 这个初始化对吗？
CircleQueue<int64_t, 10> Timer::time_gap_queue;
CircleQueue<int64_t, 10> Timer::time_gap_filter_queue;

Timer::Timer() {
    // 设置起始值
    gettimeofday(&time_start, nullptr);
    // 计时器初始状态为工作模式
    is_open = true;
}

Timer::~Timer() = default;

void Timer::printTime(const std::string &message) const {
    // 启动异常提示
    if (!is_open) {
        std::cerr << "计时器未工作!\n";
        return;
    }
    /// 此时的日历时间
    timeval time_current;
    gettimeofday(&time_current, nullptr);
    /// 计时器的工作时间
    double delta_time = static_cast<double>(
        ((time_current.tv_sec - time_start.tv_sec) * 1000 +
         (time_current.tv_usec - time_start.tv_usec) / 1000.0));

    std::cout << message << ": " << (1000.0 / delta_time) << "fps" << std::endl;
}

double Timer::getTime() const {
    // 启动异常提示
    if (!is_open) {
        std::cerr << "计时器未工作!\n";
        return 0.0;
    }
    /* 返回模块所需时间，用当前时间减去设定的开始时间，最后转为毫秒级，
       CLOCKS_PER_SEC为一秒钟内CPU运行的周期数 */
    /// 此时的日历时间
    timeval time_current;
    gettimeofday(&time_current, nullptr);
    // 返回模块所需时间，tv_sec 和 tv_usec
    // 都用当前时间减去设定的开始时间，并转为毫秒级 注意1000“.0”！
    return static_cast<double>(
        ((time_current.tv_sec - time_start.tv_sec) * 1000 +
         (time_current.tv_usec - time_start.tv_usec) / 1000.0));
}

int64_t Timer::getTimeGap (int64_t &base_mcu_time) 
{
    // 启动异常提示
    if (!is_open) {
        std::cerr << "计时器未工作!\n";
        return 0;
    }
    // 计算当前电控时刻和视觉时刻的时间差
    double current_visual_time = getTime();
    int64_t time_gap = base_mcu_time - static_cast<int64_t>(current_visual_time);
    // 存入时间差队列
    time_gap_queue.push(time_gap);
    // 计算滤波
    int64_t sum = 0;
    for (int i = 0; i < time_gap_queue.length; ++i) 
    {
        sum += time_gap_queue.values[i];
    }
    int64_t time_gap_filter = static_cast<int64_t>(sum / (time_gap_queue.length * 1.0));
    // 存入时间差滤波结果队列
    time_gap_filter_queue.push(time_gap_filter);

    // 返回最新计算出的时间差滤波结果
    return time_gap_filter;
}

int64_t Timer::getMCUTime() {
    // 启动异常提示
    if (!is_open) {
        std::cerr << "计时器未工作!\n";
        return 0;
    }
    // 当前视觉时刻
    double current_visual_time = getTime();
    // 当前最新计算的时间差滤波
    int64_t newest_time_gap_filter;
    time_gap_filter_queue.back(newest_time_gap_filter, 1);
    // 当前计算的出的电控时刻
    int64_t current_mcu_time = static_cast<int64_t>(current_visual_time) + newest_time_gap_filter;
    return current_mcu_time;
}

void Timer::restart()
{
    // 重置计时器的零点 
    gettimeofday(&time_start, nullptr);
    // 计时器初始状态为工作模式
    is_open = true; 
}