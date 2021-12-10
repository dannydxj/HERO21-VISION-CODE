// 追踪模式下装甲板选择代码
#include "aimbot.h"
/*
1. 如何实现追踪？
    优先选择前后帧装甲板中心点距离最小的装甲板
2. 关于old_armor
    old_armor在onExit()时被target_armor赋值,
    而target_armor只有在有装甲板才会被更新
    因为最多允许丢失k帧，所以即使在上一帧或者前几帧丢失装甲板后old_armor不会有太大的滞后.(最多滞后k帧)
    丢失超过k帧进入搜索模式，不会用到old_armor,搜索成功更新装甲板/进行追踪模式/此时old_armor已更新
    搜索失败继续搜索，直到搜索成功。
3. 如何上一帧不使用数字识别（默认装甲板数字为零）而当前帧使用数字识别是否会成错误过滤？
    不会，追踪模式下的数字识别开启条件增加了old_armor的数字不为零的操作。
    追踪模式下的数字识别仅用作过滤要追踪的装甲板（可以理解追踪old_armor）,
    如果d_armor数字为零，还过滤个锤子
4. 小陀螺模式下,需要切断追踪,否则运动预测带来的影响是描边大师.
   方案是在检测到运动速度快速时,如果识别到图像高度符合,面积相差控制在20%,两者之间的距离和装甲板宽度的比例符合机器人的比例
   为同一个数字, 注意roi不能取得太小,或者是不取roi
   设置一个标志位,使得在反小陀螺时反切换操作不工作// TODO先不做,因为反切换的阈值比较小
   需要将历史点删掉,也就是判断切换
   反陀螺算法的总体实现就是找待切换的装甲板,找到就切换,找不到就打击track到的装甲板
*/

bool AimBot::tryTrackArmor(std::vector<Armor> &armors, int mcu_number, int dir, double speed) 
{
    /* 根据装甲板类型进行过滤*/
    for (auto iter = armors.begin(); iter != armors.end();) 
    {
        if (iter->isBigArmor != old_armor.isBigArmor) 
        {
            iter = armors.erase(iter);
        } else 
        {
            ++iter;
        }
    }
    if (armors.empty()) 
        return false;

    /* 根据装甲板的高度筛选,TODO,待测试*/
    for (auto iter = armors.begin(); iter != armors.end();)
    {
        if (fabs(iter->armor_rect.center.y - old_armor.armor_rect.center.y) >  2 *  old_armor.armor_rect.size.height)

        {
            iter = armors.erase(iter);
        }
        else
        {
            ++iter;
        }
    }

    /* 根据装甲板与上一帧装甲板图像坐标距离降序排序， 实现追踪目的*/
    for (auto &armor : armors) 
    {
        armor.offset_distance = Util::distance(armor.armor_rect.center, old_armor.armor_rect.center);
    }
    std::sort(armors.begin(), armors.end(), Armor::setOffsetDistance);

    /* 数字识别,仅作过滤目的*/
    /* 注意!!! 在上一帧装甲板数字不为零时启用追踪特定数字 */
    if (USE_MODEL && mcu_number != EnemyNumber::EXIT_MODEL &&
        model_work_status == true && old_armor.classifier_num != 0) 
    {
        for (auto iter = armors.begin(); iter != armors.end();) 
        {
            iter->setNumberImg(mcu_number);
            int classifier_number = 0;
            classifier_number = classifier_sj(iter->number_img);
            iter->classifier_num = classifier_number;
            /* 符合当前追踪装甲板 */
            bool condition = (classifier_number == curr_number);
            if (!condition) {
                iter = armors.erase(iter);
            } else {
                ++iter;
            }
        }
    }
    /* 注意!!! 在上一帧装甲板数字为零时数字识别仅作过滤作用 */
    else if (USE_MODEL && mcu_number != EnemyNumber::EXIT_MODEL &&
             old_armor.classifier_num == 0 && model_work_status == true)
    {
        for (auto iter = armors.begin(); iter != armors.end();) {
            iter->setNumberImg(mcu_number);
            int classifier_number = 0;
            classifier_number = classifier_sj(iter->number_img);
            iter->classifier_num = classifier_number;
            /* 排除误识别装甲板或者是工程 */
            bool condition = (classifier_number != 0 &&
                              classifier_number != 2 && classifier_number != 9);
            if (!condition) {
                iter = armors.erase(iter);
            } else {
                ++iter;
            }
        }
    }

    Armor track_armor; // 存储追踪装甲板
    if (!armors.empty() && !judgesArmorSwitched(armors.at(0), old_armor))
    {
        track_armor = armors.at(0);
        track_status = true;
    }

    // if ((speed > GYRO_LOW_SPEED_THRESH) && (dir != 0) && old_armor.distance < 3.0)
    if (0)
    { // 超过低速速度阈值,但不超过高速阈值,启动反小陀螺算法
        if (armors.empty())
        {
            flag_switch_armor = 0;
            return false;
        }
        for (auto iter = armors.begin(); iter != armors.end();)
        {
            bool cond1 = (iter->armor_rect.size.area() - old_armor.armor_rect.size.area()) < old_armor.armor_rect.size.area() * 0.35;
            bool cond2 = (iter->armor_rect.center.x - old_armor.armor_rect.center.x) * dir < 0;
            bool cond3 = judgesArmorSwitched(*iter, old_armor);
            if ((!cond1) || (!cond2) || (!cond3)) 
            {
                iter = armors.erase(iter);
            }else{
                ++iter;
            }
        }

        if (armors.size() > 1)
        { //反小陀螺的待切换装甲板超过1个,说明该识别不准确,还是要击打track到的装甲板
            if (track_status){
                armors.at(0) = track_armor;
            }
        }else if (armors.size() == 0){ // 反小陀螺的待切换装甲板超过1个,说明没识别到,还是要击打track到的装甲板
            if (track_status){
                armors.emplace_back(track_armor);
            }
        }else{ // 识别到待切换的装甲板
            anti_gyro_status = true;
            // do thing
        }
    }

    if (anti_gyro_status) // 反陀螺模式
    {
        flag_switch_armor = 1;
        return true;
    }else if (track_status) // 追踪同一个装甲板
    {
        flag_switch_armor = 0;
        return true;
    }else {
        flag_switch_armor = 0; // 未找到目标
        return false;
    }
}

bool AimBot::judgesArmorSwitched(const Armor &curr_armor, const Armor &old_armor) 
{
    /* 装甲板像素距离 */
    double dis = Util::distance(curr_armor.armor_rect.center, old_armor.armor_rect.center);
    if (dis > curr_armor.armor_rect.size.height * 2) 
    {
        return true;
    } else {
        return false;
    }
}
