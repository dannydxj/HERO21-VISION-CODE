#include "runepredictor.h"

/////////////////////////////////////////匀速预测//////////////////////////////////////////////////////////////////////////
bool RunePredictor::PredConstVelocity(const ConstRuneData &rune_data, int64_t clock_now, double v, Target &target)
{
    /* 由clock_now计算时间差，单位s,是个小量 */
    double delta_clock = (clock_now - rune_data.clock) / 1000.0;
    /* 根据最近的一帧装甲板数据计算子弹飞行时间 */
    double tmp_flight_time =  Util::computFlightTime(rune_data.target, v);
    if (tmp_flight_time == ERROR)
    { /* 计算飞行时间失败 */
        return false;
    } 
    /* 然后根据拟合直线预测出目标的位置 */
    /* 该直线已经经过验证 */
    Target tmp_target = getTargetFromConstRuneData(rune_data, tmp_flight_time + delta_clock);
    /* 根据预测的位置重新计算弹道时间 */
    double flight_time = Util::computFlightTime(tmp_target, v);
    if (flight_time == ERROR){
        return false;
    } 
    /* 根据预测的弹道时间重新预测目标位置 */  
    /* 如果能够迭代多次，就会得到理想值，但是没必要 */
    target = getTargetFromConstRuneData(rune_data, flight_time + delta_clock);   
    return true;
}

Target RunePredictor::getTargetFromConstRuneData(const ConstRuneData &rune_data, double delta_clock)
{
    // 获得最近一帧点对应的角位置,单位度
    cv::Point2d center = cv::Point2d(rune_data.x, rune_data.z);
    cv::Point2d last_target_point = cv::Point2d(rune_data.target.x, rune_data.target.z);
    double last_angle = Util::getpolarAngle(center, last_target_point);
    // 计算预测的角位置(0 - 360, 逆时针为正)，需要做跨圈处理
    double target_point = last_angle + PALSTANCE * rune_data.Dir * delta_clock;
    if (target_point > 360.0){
        target_point -= 360.0;
    } else if (target_point < 0.0){
        target_point += 360.0;
    }
    Target result_target;
    result_target.y = STANDARD_LENGTH;
    result_target.x = center.x + rune_data.radius * cos(target_point / 180.0 * Util::PI);
    result_target.z = center.y + rune_data.radius * sin(target_point / 180.0 * Util::PI);
    return result_target;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////变速预测/////////////////////////////////////////////////////////////////
bool RunePredictor::PredPivVelocity(const PivRuneData &rune_data, int64_t clock_now, double v, Target &target)
{
    if (rune_data.speed_clock == 0)
    {
        return false;
    }else 
    {
        /* 由clock_now计算时间差，单位s,是个小量 */
        double delta_clock = (clock_now - rune_data.clock)/1000.0;
        /* 根据最近的一帧装甲板数据计算子弹飞行时间 */
        double tmp_flight_time =  Util::computFlightTime(rune_data.target, v);
        if (tmp_flight_time == ERROR)
        { /* 计算飞行时间失败 */
            return false;
        } 
        /* 然后根据拟合直线预测出目标的位置 */
        /* 该直线已经经过验证 */
        Target tmp_target = getTargetFromPivsRuneData(rune_data, tmp_flight_time + delta_clock);
        /* 根据预测的位置重新计算弹道时间 */
        double flight_time = Util::computFlightTime(tmp_target, v);
        if (flight_time == ERROR)
        {
            return false;
        } 
        /* 根据预测的弹道时间重新预测目标位置 */  
        /* 如果能够迭代多次，就会得到理想值，但是没必要 */
        target = getTargetFromPivsRuneData(rune_data, flight_time + delta_clock);   
        return true;
    }
}

Target RunePredictor::getTargetFromPivsRuneData(const PivRuneData &rune_data, double clock)
{
    // 计算转动增加的角度值,弧度值，注意需要考虑转到方向
    double delta_angle = rune_data.Dir * (1.305 * clock + (0.785 / 1.884) * (1 - cos(1.884 * clock))) * 180.0 / Util::PI;
    // 计算初始的角度值,弧度
    double origin_angle = Util::getpolarAngle(cv::Point2d(rune_data.target.x, rune_data.target.z), cv::Point2d(rune_data.x, rune_data.z));
    double target_angle = (delta_angle + origin_angle); 
    if (target_angle > 360.0){
        target_angle -= 360.0;
    }else if (target_angle < 0.0) 
    {
        target_angle += 360.0;
    }
    Target tmp_target;
    tmp_target.y = STANDARD_LENGTH;
    tmp_target.x = rune_data.x + rune_data.radius * cos(target_angle / 180.0 * Util::PI);
    tmp_target.z = rune_data.z + rune_data.radius * sin(target_angle / 180.0 * Util::PI);
    return tmp_target;
}