#include <iostream>

#include <iostream>
#include <math.h>
#include <string>

#include "circlequeue.h"
#include "types.h"
#include "util_func.h"
#include "base.h"

/*自瞄预测状态*/
enum PredStatus 
{
    EMPTY_DATA = 0,  // 无用于运动预测的数据点
    FEW_DATA,    // 少量数据点
    ENOUGH_DATA  // 足量数据点
};

/* 拟合运动的描述 */
enum MotionDescription
{
    STABLE,        // 低速运动
    VIOLENT,       // 高速运动
};

class ArmorPredictor {
public:
    /// 用于debug的image
    cv::Mat debug_image;

    /// 数据点个数
    int data_status;

    /// 开火指令
    bool fire = false;

    /// 是否进行爆发发弹
    bool burstAim = false;

    /// 预测yaw
    double pred_yaw;

    /// 预测pitch
    double pred_pitch;

    /// 预测yaw角速度
    double yaw_palstance;

    /// 预测pitch角速度
    double pitch_palstance;

    /// yaw精度
    double yaw_resolution;

    /// pitch精度
    double pitch_resolution;

    /// 长周期预测的直线 (数据超过50)
    Line l_line;

    /// 短周期预测的直线 (少于50个点)
    Line s_line;

    /// 预测坐标和时间轴上的时间
    Target pred_target;

    /// 弹道飞行时间
    double flight_time;

    /// 疑似噪声点数
    int noise_cnt = 0;
public:
    /// 拟合预测直线
    void run(cv::Mat &image ,CircleQueue<Target, ARMOR_PRED_LNUM> &data, int64_t clock_now, double v);
    
    /**
     * @brief 丢掉过时的点，阈值是300ms
     * @details 该函数其实限制了在拟合直线的时候的所有点都为最近的300ms
     */
    void dropOutdatedData(CircleQueue<Target, ARMOR_PRED_LNUM> &data, int64_t clock_now);

    /**
     * @brief 处理无点的情况
     */
    void handleEmptyData();

    /**
     * @brief 处理少数点的情况
     */
    void handleFewData(const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v);

    /**
     * @brief 处理多数点的情况
     */
    void handleEnoughData(const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v);

    /**
     * @brief   对拟合的运动直线做判断
     * @details 在各个方向上如果拟合出来的速度超过设定值，认为该拟合直线失败, 不更新该直线
     * @details 对直线度进行判断,未做,TODO
     */
    bool verdictLine(const Line &line, const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v);

    /**
     * @brief 对速度进行限幅
     */
    void limitSpeed(Line &line);

    /**
     * @brief 判断拟合的直线能否比较准确的预测近几帧的数据
     * @datails 如果近5帧平均残差超过10cm，则认为该直线预测效果较差
     * @deatils 该函数只有在成功预测后在会被调用
     */ 
    double computResidual(const Line &line, const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v);

    /**
     * @brief 判断拟合的直线是否具有实时性
     * @details RT : real times
     * @details 如果直线不具有实时性，设置搜索目标失败
     */
    bool judgeRTLine(Line &line, int64_t clock);

    /**
     * @brief 为了防止数据跳动,做一下滤波
     * @details 如果数据点与运动拟合的数据点相差过大, cnt增1, cnt不超出阈值, 则丢掉
     * @details cnt超出阈值, 将历史点清空,并重新拟合
     * @returns 0:丢掉点 1:接受点 2:把历史点清空
     */
    int judgeInComePointAcceptable(Target &target, double v, int64_t image_clock);

    /**
     * @brief 从直线取点
     */
    Target getTmpPredtargetFromLine(const Line &line, double delta_time);

    /**
     * @brief 获得大地系中的预测三维点
     */
    bool getPredtarget(double v, int64_t clock);

    /**
     * @brief 重载函数
     */    
    bool getPredtarget(double v, int64_t clock_now, Target &target);

    /**
     * @brief 计算云台yaw角速度
     * @param target 预测的三维点
     */
    double getYawPalstance(const Target &target, double x_velocity, double y_velocity);

    /**
     * @brief 计算云台pitch角速度
     * @details 如果需要击打，可能会出现找不到目标（最多允许2帧，所以可以忽略滞后的影响），但可以运动预测推算的目标三维点
     * @param target 预测的三维点
     */
    double getPitchPalstance(const Target &target, const Line &line, double flight_time, double v);

    /**
     * @brief 获得yaw的精度，为装甲板的半角宽
     * @param target为预测点
     */
    double getYawResolution(const Target &target, const bool isBigArmor);
    
    /**
     * @brief 获得pitch的精度，为装甲板的半角高
     * @oaram target为预测点
     */
    double getPitchResolution(const Target &target);

    /**
     * @brief 对通信的数据包进行赋值
     * @details 只有在用直线正确预测后才会调用该函数
     */
    void setPredOKSendPack(SendPack &send_pack);

    /**
     * @brief 对通信的数据包进行赋值
     * @details 适合未积累起足够量的数据用来拟合直线的情况，使云台响应更加及时
     * @details 模板函数，声明与实现放在同一个文件中
     */
    template<typename T>
    void setPredErrorSendPack(T &t, SendPack &send_pack, double v)
    {
        send_pack.burstAim = this->burstAim; // 提前调整到高射频
        send_pack.fire = false;
        send_pack.target_found = true;
        int end = (t.tail + t.size - 1) % t.size;
        send_pack.pred_yaw = Util::getPredYaw(t.values[end]);
        send_pack.pred_pitch = Util::getPredPitch(t.values[end], v);
        send_pack.yaw_resolution = getYawResolution(t.values[end], true);
        send_pack.pitch_resolution = getPitchResolution(t.values[end]);
        send_pack.yaw_palstance = 0.0;
        send_pack.pitch_palstance = 0.0;
    }

    /**
     * @brief 在debug_image 上展示图片
     */
    void displayInfo();
};