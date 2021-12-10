#include "aimbot.h"

static void adjustArmorRect(cv::RotatedRect &rect);
static bool checkLightbarSize(const std::vector<Lightbar> &lightbars);
static void scoreArmor(std::vector<Armor> &armors);

void AimBot::matchArmors(const std::vector<Lightbar> &lightbars, std::vector<Armor> &armors) 
{
    // 注意要检查是否是empty，因为begin()和end()是不检查是否为空的
    if (!checkLightbarSize(lightbars)) 
        return;

    for (auto lbar = lightbars.begin(); lbar != lightbars.end(); ++lbar) {
        auto rbar = lbar;
        for (++rbar; rbar != lightbars.end(); ++rbar) 
        {
            //------------------像素距离-----------------------------------
            float delta_width = fabs(rbar->rect.center.x - lbar->rect.center.x);
            float delta_height =
                fabs(rbar->rect.center.y - lbar->rect.center.y);
            // 装甲板x距离过近
            if (delta_width < lbar->rect.size.height) continue;
            armor_printf("111\n");
            // 装甲板x距离过远
            if (delta_width >
                lbar->rect.size.height * (MAX_ASPECT_BIG_RATIO + 1))
                break;
            armor_printf("222\n");
            // 装甲板y距离过远
            if (delta_height > 150.0) 
                continue;
            armor_printf("333\n");

            //---------------装甲板比例--------------------------------------
            double armor_width =
                sqrt(delta_width * delta_width + delta_height * delta_height);
            double armor_height =
                (lbar->rect.size.height + rbar->rect.size.height) / 2;
            double armor_ratio = armor_width / armor_height;
            // 默认小装甲板
            bool isBigArmor = false;
            if (armor_ratio > 3.7) 
            {
                isBigArmor = true;
            }
            if (isBigArmor) 
            {
                if (armor_ratio < MIN_ASPECT_BIG_RATIO || armor_ratio > MAX_ASPECT_BIG_RATIO)
                {
                    continue;
                }
            } 
            else 
            {
                if (armor_ratio < MIN_ASPECT_SMALL_RATIO || armor_ratio > MAX_ASPECT_SMALL_RATIO)
                {
                    continue;
                }
            }
            armor_printf("444\n");

            //--------------装甲板三维空间距离------------------
            double armor_distance = camera_fy * 0.055 / armor_height;
            if (armor_distance > 10.0) // 注意,哨兵不一样 
                continue;
            armor_printf("555\n");

            //--------------装甲板倾斜角度-----------------------
            double armor_angle = fabs(atan2(delta_height, delta_width) * 180 / Util::PI);
            if (armor_angle > MAX_ARMOR_ANGLE) continue;
            armor_printf("666\n");

            //--------------两灯条长度比-----------------------------------------
            double lbar_len_ratio =
                lbar->rect.size.height / rbar->rect.size.height;
            lbar_len_ratio = ((lbar_len_ratio > 1.0) ? lbar_len_ratio
                                                     : (1.0 / lbar_len_ratio));
            if (lbar_len_ratio > MAX_LENGTH_RATIO) continue;
            armor_printf("777\n");

            //-------------两灯条角度差------------------------------------------------
            double lbar_delta_angle = abs(lbar->angle - rbar->angle);
            if (lbar_delta_angle > MAX_LIGHTBAR_DELTA) continue;
            armor_printf("888\n");

            //------------防止出现带有中间灯条的误匹配----------------------------------
            double armor_area =
                (lbar->rect.size.area() + rbar->rect.size.area()) * 0.5;
            bool valid = true;
            auto temp_iter = rbar;
            std::advance(temp_iter, -1);
            if (lbar != temp_iter) {
                auto lbar_mid = lbar;
                for (++lbar_mid; lbar_mid != rbar; lbar_mid++) 
                {
                    double k = delta_height / delta_width;
                    int target_y = int( k * (lbar_mid->rect.center.x - lbar->rect.center.x) + lbar->rect.center.y);
                    // 第二个条件是为了防止弹丸在装甲板中心的影响,
                    // 第三个条件是防止数字的影响
                    if (abs(lbar_mid->rect.center.y - target_y) < 20 &&
                        abs(lbar_mid->rect.size.height - armor_height) <
                            armor_height / 5.0 &&
                        abs(lbar_mid->rect.size.area() - armor_area) <
                            armor_area / 5.0) {
                        valid = false;
                        break;
                    }
                }
            }
            if (!valid) continue;
            armor_printf("999\n");

            Armor armor;
            armor.distance = armor_distance;
            armor.src = debug_image;
            armor.assignMemberVariables(*lbar, *rbar);
            armors.emplace_back(armor);
        }
    }
}

static bool checkLightbarSize(const std::vector<Lightbar> &lightbars) {
    if (lightbars.empty() || lightbars.size() == 1)
        return false;
    else
        return true;
}
