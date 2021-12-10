#include "workspace.h"

void Workspace::RuneFunc() 
{
    if (!USE_CAN)
    {
        rune_detector.run(curr_image_object, work_msg);
    }else
    {
        rune_detector.run(curr_image_object, work_msg);
        if (work_msg.mode == Mode::MODE_SMALLRUNE)
        {
            rune_descriptior.runSmallRune(curr_image_object, work_msg, rune_detector.todo_candidate_rects, rune_detector.energy_yaw);
        }else{
            rune_descriptior.runBigRune(curr_image_object, work_msg, rune_detector.todo_candidate_rects, rune_detector.energy_yaw);
        }
    }
}