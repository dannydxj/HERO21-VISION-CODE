#ifndef RUNE_PREDICTOR_H
#define RUNE_PREDICTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "types.h"
#include "util_func.h"

class RunePredictor
{
public:
    /// 能量机关标准距离,单位m
    constexpr static double STANDARD_LENGTH = 6.9;

    /// 能量机关标准半径,单位m
    constexpr static double STANDARD_RUNE_RADIUS = 0.70;

    /// 小能量机关转速,单位 度/s
    constexpr static double  PALSTANCE = 60.0;

    /// 大能量机关转速
public:

public:
    /* 匀速能量机关预测主函数 */
    bool PredConstVelocity(const ConstRuneData &rune_data, int64_t clock_now, double v, Target &target);

    Target getTargetFromConstRuneData(const ConstRuneData &rune_data, double clock);

    /* 变速能量机关预测主函数 */
    bool PredPivVelocity(const PivRuneData &runeData, int64_t clock_now, double v, Target &target);

    Target getTargetFromPivsRuneData(const PivRuneData &rune_data, double clock);

};

#endif

