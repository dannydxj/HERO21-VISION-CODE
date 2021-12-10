// 主图像处理线程
#include "workspace.h"

void Workspace::imageProcessingFunc() 
{
    if (!USE_CAMERA) 
    {
        cap.open(VIDEO_PATH);
        totalFrame = cap.get(CV_CAP_PROP_FRAME_COUNT);//获取总帧数
    }
    
    fstream fps_recorder;
    fps_recorder.open("../logs/fps.txt", ios::out);
    Timer getRealfpstimer;
    static int cnt = 0;

    while (true) 
    {
        Timer process_timer; 
        if (getImgFromBuffer()) 
        {
            cnt++; //记录实际处理图片的cnt
            if (cnt == 100) // 每100帧计算一下平均时间
            { 
                cnt = 0;
                double duration = getRealfpstimer.getTime() - process_timer.getTime();
                fps_recorder << "fps:" << static_cast<double>(100000.0 / duration) << std::endl;
                getRealfpstimer.restart();
            }
            // 设置模式/颜色/弹速/右键是否按下标志
            msg_mutex.lock();
            int last_mode = work_msg.mode;
            work_msg = original_msg;
            msg_mutex.unlock();
            setMode(); /* 设置工作模式 */
            setCamera(last_mode, work_msg.mode); /* 根据模式切换选择是否重置相机 */
            setColor();  /* 设置工作颜色 */

            switch (work_msg.mode) 
            {
                case Mode::MODE_ARMOR2:
                    armorFunc();
                    break;
                case Mode::MODE_SMALLRUNE:
                case Mode::MODE_BIGRUNE:
                    RuneFunc();
                    break;
                case Mode::MODE_SENTINEL:
                    hitSentinelFunc();
                default:
                    break;
            }
            if (SHOW_IMAGE) 
            {
                showImage();
            }
            if (!USE_CAMERA) 
            {
                cv::waitKey(0);
                // VideoAnalyse(image_original);
            }
            if (RUNNING_TIME) 
            {
                process_timer.printTime("图像处理帧率：");
            }
            std::cout << "-------------------------------------------------" << std::endl;
        }else{
            continue;
        }

    }
}

void Workspace::setMode() 
{
    switch (MODE) 
    {
        case MODE_AUTO: 
            {
                break;
            }
        case MODE_ARMOR2:
        case MODE_SMALLRUNE:
        case MODE_BIGRUNE:
        case MODE_SENTINEL:
            {
                work_msg.mode = MODE;
                break;
            }
        default:
            LOGE("error MODE!");
            break;
    }
}

void Workspace::setColor()
{
    switch (ENEMY_COLOR) 
    {
        case COLOR_AUTO:
            break;
        case COLOR_RED:
        case COLOR_BLUE:
            work_msg.enemy_color = ENEMY_COLOR;
            break;
        default:
            LOGE("error COLOR");
            break;
    }
}

void Workspace::setCamera(int last_mode, int curr_mode)
{
    switch (curr_mode)
    {
    case Mode::MODE_ARMOR2:
            {
                if (last_mode != Mode::MODE_ARMOR2) 
                {
                    camera->CameraSetForAromr();
                }
                break;
            }
    case Mode::MODE_SENTINEL:
        {
            if (last_mode != Mode::MODE_SENTINEL)
            {
                camera->CameraSetForSentinel();
            }
            break;
        }
    case Mode::MODE_SMALLRUNE:
    case Mode::MODE_BIGRUNE:
        {
            if ((last_mode != Mode::MODE_SMALLRUNE ) && (last_mode != Mode::MODE_BIGRUNE))
            {
                camera->CameraSetForRune();
                rune_detector.clear();
                rune_descriptior.clear();
            }
            break;
        }
    default:
        break;
    }
}




