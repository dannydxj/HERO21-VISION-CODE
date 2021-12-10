#include "workspace.h"

void Workspace::hitSentinelFunc()
{
    if (!USE_CAN) 
    {
        curr_image_object.source_image = image_original;
        sentinel_hunter.run(curr_image_object, work_msg.enemy_color);
    }  
    else
    {
        if (sentinel_hunter.run(curr_image_object, work_msg.enemy_color)) // 找到目标
        {
            target = sentinel_hunter.ptz_target;    
            target.clock = curr_image_object.pose.mcu_time; /* 给坐标以时钟,时间单位注意是ms */
            {
                std::unique_lock<std::mutex> locker(pred_mutex);
                int flag = armor_predictor.judgeInComePointAcceptable(target, work_msg.bullet_speed, curr_image_object.pose.mcu_time);
                if (flag == 0)
                {
                    // do nothing
                }else if (flag == 1)
                {
                    his_coords.push(target);
                }else if (flag == 2)
                {
                    his_coords.clear();
                    his_coords.push(target);
                }
            }
            armor_predictor.run(image_original, his_coords, workspace_timer.getMCUTime(), work_msg.bullet_speed); // 拟合直线
        }
    }
}