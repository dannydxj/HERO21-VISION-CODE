// 搜索模式下装甲板选择代码
#include "aimbot.h"

static void listArmorsInOrder(std::vector<Armor> &armors);

bool AimBot::trySearchArmor(std::vector<Armor> &armors, int mcu_number, int mcu_color) 
{
    std::vector<Armor> tmp_big_armors;
    if (armors.empty()) 
    {
        return false;
    }

    if (USE_MODEL && mcu_number != EnemyNumber::EXIT_MODEL &&  model_work_status == true) 
    {
        if (armors.size() >= MAX_CANDIDATE_NUM)  // 限制候选装甲板数量
        {
            armors.resize(MAX_CANDIDATE_NUM);
        }
        for (auto iter = armors.begin(); iter != armors.end();) 
        {
            iter->setNumberImg(mcu_number);
            int classifier_number = 0;
            // enemy_number = EnemyNumber::SHOOT_BUILDING;
            // //用于测试darknet二分类
            if (mcu_number != EnemyNumber::SHOOT_BUILDING) 
            {
                classifier_number = classifier_sj(iter->number_img);
                iter->setPriority(classifier_number, mcu_number);
                /* 颜色不满足条件 */
                bool condition = ((mcu_color == COLOR_RED && classifier_number >= 8) ||  (mcu_color == COLOR_BLUE && classifier_number <= 7));
                /* 丢弃误识别装甲板或者是工程机器人 */
                bool ToDiscard = (classifier_number == 0 || classifier_number == 9 || classifier_number == 2);
                if (ToDiscard || !condition) 
                {
                    if (iter->isBigArmor) // 大装甲板由于网络模型不能准确识别，需要单独处理
                    {       
                        tmp_big_armors.emplace_back(*iter);
                        iter = armors.erase(iter); 
                    }
                    else {
                        iter = armors.erase(iter); 
                    }
                } else {
                    ++iter;
                }
            } 
        }
    }

    /* 首先对成功识别数字的装甲板（大、小都有）进行排序 */
    if (!armors.empty())
    {
        sort(armors.begin(), armors.end(), Armor::priorityComparator);
    }

    /* 将大装甲板放进armors的最后 */ 
    if (!tmp_big_armors.empty())
    {
        sort(tmp_big_armors.begin(), tmp_big_armors.end(), Armor::priorityComparator); // 首先对未成功识别数字的大装甲板排序
        for (auto &tmp_big_armor : tmp_big_armors)
        {
            armors.emplace_back(tmp_big_armor);
        }
    }
    if (armors.empty()) 
    {
        return false;
    }else{
        return true;
    }
}

void AimBot::sortArmors(std::vector<Armor> &armors) 
{
    if (armors.empty()) 
    {
        return;
    }
    sort(armors.begin(), armors.end(), Armor::scoreComparator);
    // listArmorsInOrder(armors);
}

static void listArmorsInOrder(std::vector<Armor> &armors) {
    for (auto armor : armors) {
        std::cout << "error: " << armor.sum_error_score << "|";
    }
    std::cout << std::endl;
}