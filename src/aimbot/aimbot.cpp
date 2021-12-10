#include "aimbot.h"

#include <cmath>
#include <iterator>

#include "timer.h"
#include "util_func.h"

// flag_switch_armor 是一个非常严格的条件， 一旦为真，预测程序会直接删掉历史点
// 因此，需要在漏识别/装甲板闪灭的情况下， flag_switch_armor 为假
// 换言之，
// 只有在search模式下搜索到的装甲板与历史装甲板像素距离过大的时候直接扔掉 或者是
// 反小陀螺算法下
// 追踪模式实现追踪同一个机器人，装甲板切换以flag_switch_armor为标志

bool AimBot::run(cv::Mat &image, const ReadPack &mcu_data, int dir,
                 double speed) {
    onEnter();

    if (mcu_data.enemy_number != 0 ||
        mcu_data.isRightMouseButtonClicked) /* 电控端追踪数字时强制启动搜索模式 */
    {
        this->curr_state = AimState::SEARCH_STATE;  // search 状态也可以有roi
        LOGW("restart searching");
    }
    if (curr_state == AimState::SEARCH_STATE) 
    {
        targetSearched = searchArmor(image, mcu_data);
        if (targetSearched) {
            if (judgesArmorSwitched(armors.at(0), old_armor)) 
            {
                flag_switch_armor = 1;
            } else {
                flag_switch_armor = 0;
            }
        } else {
            flag_switch_armor = 0;
        }
    } else {
        trackSuccess = trackArmor(image, mcu_data, dir, speed);
        flag_switch_armor = 0;
        if (trackSuccess) {  //
            targetTracked = true;
            lost_cnt = 0;
        } else {
            ++lost_cnt;
            if (lost_cnt >= max_lost_cnt) {  // 丢失帧数
                targetTracked = false; /* 丢失帧数超过最大帧数，认为追踪失败，重新启动搜索模式*/
                lost_cnt = 0;
            } else {
                targetTracked = true;
            }
        }
    }
    // drawArmors(debug_image, armors);
    // drawBars(debug_image, lightbars);
    displayInfo(debug_image);
    onExit();
    if (curr_state == AimState::SEARCH_STATE) {
        return targetSearched;
    } else {
        return trackSuccess;
    }
}

bool AimBot::searchArmor(cv::Mat &image, const ReadPack &mcu_data) {
    processImage(image, mcu_data.enemy_color);            /* 图像预处理 */
    findLightbars(processed_image, mcu_data.enemy_color); /* 寻找灯条 */
    matchArmors(lightbars, armors); /* 灯条两两匹配装甲板 */
    gradeArmors(armors); /* 对匹配得到的装甲板们进行误差分数赋值 */
    sortArmors(armors); /* 根据误差分数降序排列 */
    trySearchArmor(armors, mcu_data.enemy_number, mcu_data.enemy_color); /* 数字识别过滤误识别的装甲板并排序 */
    judgeBurstAim(armors, mcu_data.enemy_number); /* 爆发式自瞄判断 */
    if (!armors.empty()) {
        target_armor = armors.at(0);
        curr_number = target_armor.classifier_num; /* 更新当前追踪数字(1.电控发来的信息
                                                      2.误差分数) */
        if (ROI_ENABLE) {
            setRoiRect(
                target_armor
                    .rect());  // 把框出装甲板的旋转矩形的包围矩形适当放大
        }
        armorFound = true;
    } else 
    {
        roi_rect = cv::Rect();
        armorFound = false;
    }
    return armorFound;
}

bool AimBot::trackArmor(cv::Mat &image, const ReadPack &mcu_data, int dir, double speed) 
{
    processImage(image, mcu_data.enemy_color);            /* 图像预处理 */
    findLightbars(processed_image, mcu_data.enemy_color); /* 寻找灯条 */
    matchArmors(lightbars, armors); /* 灯条两两匹配装甲板 */
    tryTrackArmor(armors, mcu_data.enemy_number, dir, speed); /* 追踪装甲板 */
    judgeBurstAim(armors, mcu_data.enemy_number); /* 爆发式自瞄判断 */
    if (track_status) {
        std::cout << armors.size() << std::endl;
        target_armor = armors.at(0);
        if (ROI_ENABLE) {
            setRoiRect(target_armor.rect());  // 把框出装甲板的旋转矩形的包围矩形适当放大
        }
        armorFound = true;
    } else {
        roi_rect = cv::Rect();
        armorFound = false;
    }
    return armorFound;
}

void AimBot::ApplyModelBaseOnDistance() {
    bool tmp = model_work_status;  // 记录上一次的状态
    if (armors.empty()) /* 未找到装甲板，下一帧数字识别工作状态不变,
                           历史距离清零 */
    {
        model_work_status = tmp;
    } else /* 找到装甲板 & 上一帧未找到 */
    {
        if (target_armor.distance < 0.8) 
        {
            model_work_status = false;
        } else 
        {
            model_work_status = true;
        }
    }
}

void AimBot::config() {
    this->curr_state = this->next_state = AimState::SEARCH_STATE;
    this->model_work_status = true;
}

void AimBot::onEnter() {
    this->curr_state = this->next_state;
    track_status = false;

    lightbars.clear();
    armors.clear();
}

void AimBot::onExit() {
    ApplyModelBaseOnDistance(); /* 决定下一帧是否使用数字识别 */
    old_armor = target_armor;
    /* 当前帧处于搜索装甲板状态 */
    if (this->curr_state == AimState::SEARCH_STATE) 
    {
        if (targetSearched) {
            this->next_state = AimState::TRACK_STATE;
        } else {
            this->next_state = AimState::SEARCH_STATE;
        }
    }
    /* 当前帧处于追踪装甲板状态 */
    else {
        if (targetTracked) {
            this->next_state = AimState::TRACK_STATE;
        } else {
            this->next_state = AimState::SEARCH_STATE;
        }
    }
    anti_gyro_status = false;
}

void AimBot::drawBars(cv::Mat &debug_image, std::vector<Lightbar> &bars) {
    LOGM("灯条数量%d", lightbars.size());
    for (auto &bar : bars) {
        cv::circle(debug_image, bar.rect.center, 2, cv::Scalar(0, 255, 0), -1,
                   8, 0);
    }
}

void AimBot::drawArmors(cv::Mat &debug_image, std::vector<Armor> &armors) {
    LOGM("装甲板数量%d", armors.size());
    if (armors.empty()) return;
    for (auto &armor : armors) {
        Util::drawRotatedRect(
            debug_image, armor.armor_rect,
            cv::Scalar(0, 0, 255)); /* 红色：绘制候选装甲板们 */
        // putText(debug_image, to_string(armor.armor_angle),
        // armor.armor_rect.center, cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0,
        // 255, 0), 1, 8);
    }
    if (curr_state == AimState::TRACK_STATE) {
        Util::drawRotatedRect(debug_image, old_armor.armor_rect,
                              cv::Scalar(255, 255, 255)); /* 白色：在追踪模式下绘制要追踪的装甲板（即为上一帧装甲板）
                                                           */
    }
    Util::drawRotatedRect(debug_image, armors.at(0).armor_rect,
                          cv::Scalar(0, 255, 0)); /* 绿色: 绘制待打击装甲板 */
}

void AimBot::displayInfo(cv::Mat &debug_image) {
    int width = debug_image.cols;
    int height = debug_image.rows;
    int defult_value = 1; /* 若不传入value时的默认值 */
    if (curr_state == AimState::SEARCH_STATE) {
        cv::putText(debug_image, "Searching!", cv::Point(20, 20),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
    } else {
        cv::putText(debug_image, "tracking!", cv::Point(20, 20),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    }
    if (model_work_status) {
        cv::putText(debug_image, "model yes!", cv::Point(20, 40),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    } else {
        cv::putText(debug_image, "model no!", cv::Point(20, 40),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
    }
    if (anti_gyro_status) {
        cv::putText(debug_image, "anti top yes!", cv::Point(20, 60),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
    } else {
        cv::putText(debug_image, "anti top no!", cv::Point(20, 60),
                    cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
    }
    if (!armors.empty() && armors.at(0).burstAim) {
        putText(debug_image, "burst!", armors.at(0).armor_rect.center,
                cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, 8);
    }
    if (flag_switch_armor) {  // 只有在当前帧找到装甲板才去判断是否切换目标，否则不切换目标
        putText(debug_image, "armor switched!", armors.at(0).armor_rect.center,
                cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, 8);
    }
}