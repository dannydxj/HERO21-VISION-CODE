#include "armor_predict.h"

template<typename T>
static void fitLinearMotion(T &t, Line &line);

template<typename T>
static bool fitUnidimensionalLine(T &t, Line &line, int flag);

static void judgeMotionStatus(Line &line);

void ArmorPredictor::run(cv::Mat &image, CircleQueue<Target, ARMOR_PRED_LNUM> &data, int64_t clock_now, double v) 
{
    debug_image = image;

    /* 丢掉过时点，保证了一组点的紧密性 */
    dropOutdatedData(data, clock_now);

    /* 确定预测状态 */
    if (data.length <= 3)
    {
        data_status = PredStatus::EMPTY_DATA; // 不更新直线，但是有有效点的，所以应该使云台转动
    }else if (data.length <= ARMOR_PRED_SNUM)
    {
        data_status = PredStatus::FEW_DATA;
    }else
    {
        data_status = PredStatus::ENOUGH_DATA;
    }
    /* 根据预测状态进行直线拟合， 同时更新直线的clock和target*/
    switch (data_status)
    {
    case PredStatus::EMPTY_DATA:
        handleEmptyData();
        break;
    case PredStatus::FEW_DATA:
        handleFewData(data, v);
        break;
    case PredStatus::ENOUGH_DATA:
        handleEnoughData(data, v);
    default:
        break;
    }
    displayInfo();
}

void ArmorPredictor::dropOutdatedData(CircleQueue<Target, ARMOR_PRED_LNUM> &data, int64_t clock_now)
{
    for (int i = data.head; i != data.tail; i = (i + 1) % data.size)
    {
        if (fabs(clock_now - data.values[i].clock) > 180) /*坐标点的时钟与当前的时钟相差超过180ms*/
        {
            data.popHead();
        } else
        {
            break;
        }
    }
}

void ArmorPredictor::handleEmptyData()
{
    /* 不更新直线,没有把握发弹，但是可以让云台运动到目标点 */
    fire = false;
}

void ArmorPredictor::handleFewData(const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v)
{
    fitLinearMotion(data, s_line); /* 更新短周期直线*/
    if (!verdictLine(s_line, data, v))
    {
        fire = false;
    } else {
        fire = true;
    }
    limitSpeed(l_line);    
}

void ArmorPredictor::handleEnoughData(const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v)
{
    /* 构造短周期序列 */
    CircleQueue<Target, ARMOR_PRED_SNUM> sdata; 
    for (int i = data.head; i != data.tail; i = (i + 1) % data.size) 
    {
        sdata.push(data.values[i]);
    }    
    fitLinearMotion(sdata, s_line); /* 更新短周期 */
    fitLinearMotion(data, l_line); /* 更新长周期预测 */
    if ((!verdictLine(s_line, data, v)) || (!verdictLine(l_line, data, v)))
    {
        fire = false;
    } else {
        fire = true;
    }
    limitSpeed(s_line);    
    limitSpeed(l_line);    
}

template<typename T>
static void fitLinearMotion(T &t, Line &line) 
{
    fitUnidimensionalLine(t, line, 1); // x dir
    fitUnidimensionalLine(t, line, 2); // y dir 
    fitUnidimensionalLine(t, line, 3); // z dir
    int end = (t.tail + t.size - 1) % t.size;
    line.clock = t.values[end].clock; 
    line.target = t.values[end];
    line.valid = true;
    judgeMotionStatus(line);
}

bool ArmorPredictor::getPredtarget(double v, int64_t clock_now)
{
    Line line;
    if (s_line.status == MotionDescription::STABLE)
    {
        if (judgeRTLine(s_line, clock_now))
        {
            line = s_line; 
        }else{
            return false;
        }
    }else {
        if (!judgeRTLine(l_line, clock_now)){ // 长周期时效性满足不了条件
            if (!judgeRTLine(s_line, clock_now)){  // 短周期时效性满足不了条件
                return false;
            }else{
                line = s_line;
            }
        }else {
            line = l_line;
        }
    }
    if (!line.valid){
        return false;
    }

    /* 由clock_now计算时间差，单位s,是个小量 */
    double delta_clock = (clock_now - line.clock) / 1000.0;
    /* 根据最近的一帧装甲板数据计算 子弹飞行时间 */
    double tmp_flight_time =  Util::computFlightTime(line.target, v);
    if (tmp_flight_time == ERROR){ /* 计算飞行时间失败 */
        return false;
    } 
    /* 然后根据拟合直线预测出目标的位置 */
    /* 该直线已经经过验证 */
    Target tmp_target = getTmpPredtargetFromLine(line, tmp_flight_time + delta_clock);
    /* 根据预测的位置重新计算弹道时间 */
    flight_time = Util::computFlightTime(tmp_target, v);
    if (flight_time == ERROR)
    {
        return false;
    } 
    /* 根据预测的弹道时间重新预测目标位置 */  
    /* 如果能够迭代多次，就会得到理想值，但是没必要 */
    pred_target = getTmpPredtargetFromLine(line, flight_time + delta_clock);

    /* 根据预测点计算云台设定值 */
    this->pred_yaw = Util::getPredYaw(pred_target);
    this->pred_pitch = Util::getPredPitch(pred_target, v);
    this->yaw_resolution = getYawResolution(pred_target, true);
    this->pitch_resolution = getPitchResolution(pred_target);
    this->yaw_palstance = getYawPalstance(pred_target, line.x_slope, line.y_slope);
    this->pitch_palstance = getPitchPalstance(pred_target, line, flight_time + delta_clock, v);
    return true;
}

bool ArmorPredictor::judgeRTLine(Line &line, int64_t clock_now)
{
    if (fabs(clock_now - line.clock) > 120) /* 超过100ms,该直线变为无效值 */
    { 
        return false;
    }else {
        return true;
    }
}

void ArmorPredictor::limitSpeed(Line &line)
{
    line.x_slope = (line.x_slope > 3.0) ? (3.0) : line.x_slope; 
    line.x_slope = (line.x_slope < -3.0) ? (-3.0) : line.x_slope; 
    line.y_slope = (line.y_slope > 3.0) ? (3.0) : line.y_slope;
    line.y_slope = (line.y_slope < -3.0) ? (-3.0) : line.y_slope;
}

bool ArmorPredictor::getPredtarget(double v, int64_t clock_now, Target &target)
{
    Line line;
    if (s_line.status == MotionDescription::STABLE)
    {
        line = s_line; 
    }else {
        line = l_line;
    }

    /* 首先判断直线的时效性，保证了一组点的时效性*/
    if(!judgeRTLine(line, clock_now))
    {
        return false;
    }
    /* 由clock_now计算时间差，单位s,是个小量 */
    double delta_clock = (clock_now - line.clock) / 1000.0;
    /* 根据最近的一帧装甲板数据计算子弹飞行时间 */
    double tmp_flight_time =  Util::computFlightTime(line.target, v);
    if (tmp_flight_time == ERROR) /* 计算飞行时间失败 */
    { 
        return false;
    } 
    /* 然后根据拟合直线预测出目标的位置 */
    /* 该直线已经经过验证 */
    Target tmp_target = getTmpPredtargetFromLine(line, tmp_flight_time + delta_clock);
    /* 根据预测的位置重新计算弹道时间 */
    flight_time = Util::computFlightTime(tmp_target, v);
    if (flight_time == ERROR)
    {
        return false;
    } 
    /* 根据预测的弹道时间重新预测目标位置 */  
    /* 如果能够迭代多次，就会得到理想值，但是没必要 */
    target = getTmpPredtargetFromLine(line, flight_time + delta_clock);
    return true;
}

int ArmorPredictor::judgeInComePointAcceptable(Target &target, double v, int64_t image_clock)
{
    if ((!judgeRTLine(s_line, image_clock)) || (!s_line.valid)){ // 需要保证直线是有效的
        noise_cnt = 0;
        return 1;
    }

    Target tmp_target;
    double delta_time = static_cast<double>((image_clock - s_line.clock)/1000.0);
    tmp_target = getTmpPredtargetFromLine(s_line, delta_time);
    { // 获得预测点, 并将预测点与当前点做比较,从而判断当前点的有效性
        double delta_x = fabs(tmp_target.x - target.x);
        double delta_y = fabs(tmp_target.y - target.y);
        double delta_z = fabs(tmp_target.z - target.z);
        if ( delta_x > 0.2 ||  delta_y > 0.2 || delta_z > 0.1)
        {
            noise_cnt++;
            if (noise_cnt > 6) // 超过预测
            {
                noise_cnt = 0; // 重新置零
                return 2;
            } else {
                return 0;
            } 
        }
        else
        {
            noise_cnt = 0;
            return 1;
        } 
    } 
}

Target ArmorPredictor::getTmpPredtargetFromLine(const Line &line, double delta_time) /* 从拟合直线上去寻找点, 注意输入时间单位是s */
{
    double delta_x = line.x_slope * delta_time;
    double delta_y = line.y_slope * delta_time;
    double delta_z = line.z_slope * 0; // z轴不需要做运动预测 
    
    Target tmp_target;
    tmp_target.x = line.target.x + delta_x;
    tmp_target.y = line.target.y + delta_y;
    tmp_target.z = line.target.z + delta_z;
    
    return tmp_target;
}

bool ArmorPredictor::verdictLine(const Line &line, const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v)
{
    if (fabs(line.x_slope) < 3.0 && fabs(line.y_slope) < 3.0 && fabs(line.z_slope) < 1.0)
    {
        if (computResidual(line, data, v) < 0.15) // 平均值残差小于0.15m
        {
            return true;
        }else{
            return false;
        }
    } 
    else 
    {
        return false;
    }
}

double ArmorPredictor::computResidual(const Line &line, const CircleQueue<Target, ARMOR_PRED_LNUM> &data, double v)
{
    double sum_residual = 0;
    if(data.length < 6) // 拟合点小于6，不计算
    {
        return 0.0;
    }else 
    {
        /* 选择用作计算的直线 */
        Line line;
        if (s_line.status == MotionDescription::STABLE)
        {
            line = s_line; 
        }else {
            line = l_line;
        }
        /* 判断直线的有效性和时效性 */
        Target tmp_target; 
        data.back(tmp_target, 1);
        if ((!line.valid) || (!judgeRTLine(line, tmp_target.clock)))
        {
            return 0.0;
        }else
        {
            /* 计算倒数5个点的残差 */
            for (int i = 1; i <= 5; i++)
            { // i代表从循环队列倒数第i个数
                Target tmp_real_target; 
                data.back(tmp_real_target, i);
                double delta_clock = static_cast<double>((tmp_real_target.clock - line.clock) / 1000.0);
                Target tmp_fit_target = getTmpPredtargetFromLine(line, delta_clock);
                sum_residual += (pow((tmp_real_target.x - tmp_fit_target.x), 2) + pow((tmp_real_target.y - tmp_fit_target.y), 2));
            }
            return (sum_residual / 5.0); // 返回残差均值
        }
    }
}

void judgeMotionStatus(Line &line)
{
    if ((fabs(line.x_slope) < 0.6) && (fabs(line.y_slope) < 0.6) && (fabs(line.z_slope) < 0.6))
    {
        line.status = MotionDescription::STABLE;
    }else
    {
        line.status = MotionDescription::VIOLENT;
    }
}


/* 一维直线拟合 */
template<typename T>
static bool fitUnidimensionalLine(T &t, Line &line, int flag)
{ 
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;
    double E = 0.0;
    double F = 0.0;

    for (int i = t.head; i != t.tail; i = (i + 1) % t.size) 
    {
        A += pow(((t.values[i].clock - t.values[t.head].clock) / 1000.0) , 2); // sum (x^2)
        B += ((t.values[i].clock - t.values[t.head].clock)/1000.0);            // sum (x)
        C += (get1DAccordinate(t.values[i], flag) - get1DAccordinate(t.values[t.head], flag)) 
                * ((t.values[i].clock-t.values[t.head].clock)/1000.0); // sum(x*y)
        D += (get1DAccordinate(t.values[i], flag) - get1DAccordinate(t.values[t.head], flag));
    }

    // 计算斜率a和截距b
    double a, b, temp = 0;
    if (temp = (t.length * A - B * B))  // 判断分母不为0
    {
        a = (t.length * C - B * D) / temp;
        b = (A * D - B * C) / temp;
    } else {
        a = 1;
        b = 0;
    }

    // 赋值getPredtarget
    switch (flag)
    {
        case 1:
        {
            line.x_slope = a;
            line.x_intercept = b;
            break;
        }
        case 2:
        {
            line.y_slope = a;
            line.y_intercept = b;
            break;
        }
        case 3:
        {
            line.z_slope = a;
            line.z_intercept = b;
            break;
        }
    }
}

/* 返回点集点的某个坐标 */ 
static double get1DAccordinate(const Target &target, int flag)
{
    if (flag == 1)
    {
        return target.x;
    }else if (flag == 2)
    {
        return target.y;
    }else if (flag == 3)
    {
        return target.z;
    }else 
    {
        LOGE("error acoordinate!");
        exit(1);
    }
}

double ArmorPredictor::getYawPalstance(const Target &target, double x_velocity, double y_velocity)
{
    /* 求导可得该公式 */
    double top =  y_velocity * target.x - target.y * x_velocity;
    double bottom = (pow(target.x, 2) + pow(target.y, 2));
    double result = top / bottom; 
    return (result * 180 / Util::PI);
}

double ArmorPredictor::getPitchPalstance(const Target &target, const Line &line, double flight_time, double v)
{
    // pitch的计算需要考虑抛物线的影响
    // 因为pitch的计算较为复杂，不适合用求导的方式去做
    double time_interval = 0.005; // 单位s
    Target target1 = getTmpPredtargetFromLine(line, flight_time); // 单位m
    double pitch1 = Util::getPredPitch(target1, v); 
    Target target2 = getTmpPredtargetFromLine(line, flight_time + time_interval);
    double pitch2 = Util::getPredPitch(target2, v); 
    double delta_pitch = (pitch2 - pitch1);
    double result = delta_pitch / time_interval;
    return (result * 180 / Util::PI);
}

double ArmorPredictor::getYawResolution(const Target &target, const bool isBigArmor) 
{
    double dis = sqrt(pow(target.x, 2) + pow(target.y, 2) + pow(target.z, 2));
    if (isBigArmor) 
    {
        double angle = atan2(112.5, (dis * 1000));
        return (angle * 180 / Util::PI);
    } else 
    {
        double angle = atan2(65.0, (dis * 1000));
        return (angle * 180 / Util::PI); 
    }
}

double ArmorPredictor::getPitchResolution(const Target &target) 
{
    double dis = sqrt(pow(target.x, 2) + pow(target.y, 2) + pow(target.z, 2));
    double angle = atan2(27.5, (dis * 1000));
    return (angle * 180 / Util::PI);
}

void ArmorPredictor::setPredOKSendPack(SendPack &send_pack)
{
    send_pack.target_found = true;
    send_pack.burstAim = this->burstAim;
    send_pack.fire = this->fire; //TODO
    send_pack.yaw_resolution = this->yaw_resolution;
    send_pack.pitch_resolution = this->pitch_resolution;
    send_pack.pred_yaw = this->pred_yaw;
    send_pack.pred_pitch = this->pred_pitch;
    send_pack.yaw_palstance = this->yaw_palstance;
    send_pack.pitch_palstance = this->pitch_palstance;
}

void ArmorPredictor::displayInfo()
{
    int rows = debug_image.rows;
    int cols = debug_image.cols;
    int x_start = (cols - 200);
    if (this->fire){
        cv::putText(debug_image, "fire!", cv::Point(x_start, 80), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    }else{
        cv::putText(debug_image, "stop fire!", cv::Point(x_start, 80), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(255, 255, 0));
    }

    if (this->data_status == PredStatus::EMPTY_DATA)
    {
        cv::putText(debug_image, "no data!", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
    }else if (this->data_status == PredStatus::FEW_DATA)
    {
        cv::putText(debug_image, "few data", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(255, 255, 0));
        if (s_line.status == MotionDescription::STABLE){
            cv::putText(debug_image, "stable", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        }else {
            cv::putText(debug_image, "violent", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(255, 255, 0));
        }
    }else
    {
        cv::putText(debug_image, "enough data", cv::Point(x_start, 80), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        if (l_line.status == MotionDescription::STABLE){
            cv::putText(debug_image, "stable", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        }else 
        {
            cv::putText(debug_image, "violent", cv::Point(x_start, 100), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(255, 255, 0));
        }
    }
    { // 拟合的运动速度板
        cv::putText(debug_image, "x_speed: " + std::to_string(s_line.x_slope), cv::Point(x_start, 120), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        cv::putText(debug_image, "y_speed: " + std::to_string(s_line.y_slope), cv::Point(x_start, 140), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        cv::putText(debug_image, "speed: " + std::to_string(sqrt(pow(s_line.x_slope,2)+pow(s_line.y_slope,2))), cv::Point(x_start, 160), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    }
}

