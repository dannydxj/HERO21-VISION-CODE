#include "aimbot.h"

static double get_distance_score(double distance);
static void show_error_score(Armor &armor);

void AimBot::gradeArmors(std::vector<Armor> &armors) {
    if (armors.empty()) 
        return;

    for (auto &armor : armors) 
    {
        // 距离误差分数
        armor.dis_error_score = get_distance_score(armor.distance);
        armor.sum_error_score += armor.dis_error_score;
        // std::cout << "小孔成像距离：" << distance << std::endl;

        // 权重TODO
        if (armor.isBigArmor) {
            armor.ratio_error_score = 2 * fabs(armor.ratio - 4.09);
            armor.sum_error_score += armor.ratio_error_score;
        } else {
            armor.ratio_error_score = 2 * fabs(armor.ratio - 2.363636);
            armor.sum_error_score += armor.ratio_error_score;
        }

        // 两灯条高度比造成的误差分数
        armor.bar_len_ratio_error_score =
            (armor.bar_len_ratio - 1.0) * 10.0 / (MAX_LENGTH_RATIO - 1.0);
        armor.sum_error_score += armor.bar_len_ratio_error_score;

        // 两灯条倾斜度之差不过大，因为如果是正常的匹配的灯条角度差比较小，权重要稍微高一点
        armor.bar_delta_angle_error_score =
            (armor.bar_delta_angle / (MAX_LIGHTBAR_DELTA)) * 15.0;
        armor.sum_error_score += armor.bar_delta_angle_error_score;

        // 装甲板倾斜角度误差分数
        armor.armor_angle_error_score =
            fabs(armor.armor_angle) * 15.0 / MAX_ARMOR_ANGLE;
        armor.sum_error_score += armor.armor_angle_error_score;
        // show_error_score(armor);
    }
}

void AimBot::judgeBurstAim(std::vector<Armor> &armors, int mcu_number) 
{
    if (armors.empty()) 
        return;

    for (auto &armor : armors) 
    {
        if (USE_MODEL && mcu_number != EnemyNumber::EXIT_MODEL) 
        {  // 爆发式自瞄的前提是开启数字识别
            if (armor.distance < 2 && armor.armor_angle < 10) 
            {  // 近距离和正对是触发条件
                armor.burstAim = true;
            } else {
                armor.burstAim = false;
            }
        }
    }
}

static double get_distance_score(double distance) {
    double score = 0;
    if (distance < 2.0)
        score = 0;
    else if (distance >= 2.0 && distance < 3.0)
        score = distance;
    else if (distance >= 3.0 && distance < 4.0)
        score = 8.0 * (distance - 3) + 3;
    else if (distance >= 4.0 && distance < 5.0)
        score = 12.0 * (distance - 4.0) + 11.0;
    else
        score = 4 * distance;
    return score;
}

static void show_error_score(Armor &armor) {
    std::cout << "总误差分数: " << armor.sum_error_score << std::endl;
    std::cout << "距离误差分数:" << armor.dis_error_score << "|"
              << "装甲板比例误差分数:" << armor.ratio_error_score << "|"
              << "两灯条高度比误差分数:" << armor.bar_len_ratio_error_score
              << "|"
              << "两灯条角度差误差分数:" << armor.bar_delta_angle_error_score
              << "|"
              << "装甲板角度误差分数:" << armor.armor_angle_error_score << "|"
              << std::endl;
}