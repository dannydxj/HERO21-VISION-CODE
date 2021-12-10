#include "workspace.h"

void Workspace::armorFunc() 
{ 
    /* 不通信, 只识别 */
    if (!USE_CAN) 
    {
        armor_detector.run(image_original, work_msg, 1, 0);
    }

    /* 通信，同时进行预测 */
    else 
    {
        int dir = 0;
        double speed = 0;
        // // 首先计算线速度和方向，用于反陀螺的判断, TODO
        // if (armor_predictor.s_line.valid && armor_predictor.judgeRTLine(armor_predictor.s_line, workspace_timer.getMCUTime()))
        // {
        //     double Vx = armor_predictor.s_line.x_slope;
        //     double Vy = armor_predictor.s_line.y_slope;
        //     // 计算装甲板的速度及方向
        //     speed = sqrt(pow(Vx, 2) + pow(Vy, 2));
        //     if (((workspace_timer.getMCUTime() - target.clock) < 200))
        //     {
        //         double angle = atan2(target.y, target.x);
        //         Target tmp_target;
        //         Util::transAngle(Vx, Vy, -angle);
        //         if (Vy > 0){
        //             dir = -1;
        //         }else {
        //             dir = 1;
        //         }
        //     }
        // }else {
        //     dir = 0; // 只需要这一个标志即可
        // }

        if (!armor_detector.run(image_original, work_msg, dir, speed)) // 没有找到目标
        {
            std::unique_lock<std::mutex> locker(pred_mutex);
            armor_predictor.run(image_original, his_coords, workspace_timer.getMCUTime(), work_msg.bullet_speed); // 用于拟合直线的点数会减少
            armor_predictor.burstAim = false; /* 爆发发弹标志 */
        } 
        else 
        { // 找到目标
            target_armor = armor_detector.armors.at(0); // 获得装甲板
            target_solver.armor_getCoord(armor_detector.target_armor, target);
            Util::wheel2land(target, curr_image_object); /* 转换到大地坐标系 */
            target.clock = curr_image_object.pose.mcu_time; /* 给坐标以时钟,时间单位注意是ms */
            if(target.z < 2.0) // 限制高度不超过2m
            {
                // std::unique_lock<std::mutex> locker(pred_mutex); TODO 
                armor_predictor.burstAim = target_armor.burstAim; /* 爆发发弹标志 */
                if (armor_detector.flag_switch_armor) 
                { /* 因为识别会有反切换，因此这里不用做滞回比较 */
                    his_coords.clear();
                    armor_predictor.l_line.valid = armor_predictor.s_line.valid = false;
                }
                int flag = armor_predictor.judgeInComePointAcceptable(target, work_msg.bullet_speed, curr_image_object.pose.mcu_time);
                if (flag == 2)
                {
                    armor_predictor.l_line.valid = armor_predictor.s_line.valid = false;
                    his_coords.clear();
                }
                his_coords.push(target);
            }
            armor_predictor.run(image_original, his_coords, workspace_timer.getMCUTime(), work_msg.bullet_speed); // 拟合直线
        }
    }
}