#include "workspace.h"

void Workspace::init(const cv::FileStorage &file_storage) 
{
    loadParam(file_storage);
    checkParamConflicts();
    initCommunication();
}

void Workspace::loadParam(const cv::FileStorage &file_storage) 
{
    cv::FileNode workspace = file_storage["workspace"];
    workspace["SHOW_IMAGE"] >> SHOW_IMAGE;
    workspace["TRACKBAR"] >> TRACKBAR;
    workspace["RUNNING_TIME"] >> RUNNING_TIME;
    workspace["ENEMY_COLOR"] >> ENEMY_COLOR;
    workspace["MODE"] >> MODE;
    workspace["USE_CAMERA"] >> USE_CAMERA;
    workspace["SAVE_VIDEO"] >> SAVE_VIDEO;
    workspace["USE_CAN"] >> USE_CAN;
    VIDEO_PATH = static_cast<std::string>(workspace["VIDEO_PATH"]);
    VIDEO_SAVE_PATH = static_cast<std::string>(workspace["VIDEO_SAVED_PATH"]);
}

void Workspace::checkParamConflicts() 
{
    // 保存视频必须使用相机
    if (USE_CAMERA == 0 && SAVE_VIDEO == 1) 
    {
        LOGE("wrong mode, SAVE_VIDEO need USE_CAMERA.");
        exit(1);
    }
    // 使用TRACKBAR必须展示图像
    if (SHOW_IMAGE == 0 && TRACKBAR == 1) 
    {
        LOGE("wrong mode, TRACKBAR need SHOW_IMAGE.");
        exit(1);
    }
}

void Workspace::initCommunication() 
{
    if (USE_CAN)
    {
        can_node.init();
        #ifdef USE_CAN_TO_DEBUG
            debug_can.init();
        #endif
    }else{
        return;
    }
}


void Workspace::resizeVideo(cv::Mat &src) 
{
    float length = static_cast<float>(src.cols);
    float width = static_cast<float>(src.rows);
    if (length / width > 640.0 / 480.0) 
    {
        length *= 480.0 / width;
        resize(src, src, cv::Size(length, 480));
        src = src(cv::Rect((length - 640) / 2, 0, 640, 480));
    } else 
    {
        width *= 640.0 / length;
        resize(src, src, cv::Size(640, width));
        src = src(cv::Rect(0, (width - 480) / 2, 640, 480));
    }
}

void Workspace::VideoAnalyse(cv::Mat image) 
{
    int key = cv::waitKey(50);
    if (key == 27) {
        cout << "结束播放！" << endl;
        exit(0);
    } else if (key == 's') {
        static int cnt_img = 0;
        stringstream sstream;
        sstream.str("");
        sstream << VIDEO_SAVE_PATH << cnt_img << ".jpg";
        imwrite(sstream.str(), image);
        cnt_img++;
        cv::waitKey(0);
    } else if (key == ' ') {
        cv::waitKey(0);
    }
}


bool Workspace::getImgFromBuffer()
{
    // 获取图像，两种方式（相机、视频）
    if (USE_CAMERA) 
    {
#ifdef CONSTANT_MODE_SJ
    Timer timer;
    while (camera->image_object_buffer.empty()) 
    {
        // dothing
    }
    camera->image_object_buffer.pop(curr_image_object);
    return true;
#else
        image_buffer_mutex.lock();
        if (image_object_buffer.empty()) 
        {
            image_buffer_mutex.unlock();  // 千万别而忘记解锁
            std::this_thread::sleep_for(std::chrono::microseconds(100)); // 睡眠100微秒
            // std::cout << "Second image buffer empty!" << std::endl;
            return false;
        }
        curr_image_object = image_object_buffer.back();
        image_object_buffer.clear();
        image_buffer_mutex.unlock();
        image_original = curr_image_object.source_image;
        return true;
#endif
    } 
    else 
    {
        cap >> image_original;
        static double pos = 0;
        if (video_now - video_past != 1 && video_now - video_past != 0) 
        {
            pos = (double)video_now / 100 * totalFrame;
            cap.set(CV_CAP_PROP_POS_FRAMES, pos);
        } else 
        {
            ++pos;
        }

        video_past = video_now;
        video_now = ceil((double)pos * 100 / totalFrame);
        if (image_original.empty()) 
        {
            LOGE("Video empty!");
            return false;
        }
        // 一律转到640 X 480
        resizeVideo(image_original);
        return true;
    }
}


void Workspace::showImage()  // TODO
{
    string window_name_target = "with target";
    string window_name_proc = "processed binary";
    string window_name_number = "number_image";
    cv::Mat image_draw = image_original;

	cv::namedWindow(window_name_target, CV_WINDOW_NORMAL);
	cv::namedWindow(window_name_proc, CV_WINDOW_NORMAL);
	cv::namedWindow(window_name_number, CV_WINDOW_NORMAL);


    Debugger::drawTypeValue(target, image_draw);


    if (work_msg.mode == Mode::MODE_ARMOR2) 
    {
        copyMakeBorder
        (
            armor_detector.processed_image,
            armor_detector.processed_image, 0,
            camera->frame_height - armor_detector.processed_image.rows, 0,
            camera->frame_width - armor_detector.processed_image.cols,
            cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255)
        );

        if (TRACKBAR) 
        {
            cv::createTrackbar("video_progress", window_name_target, &video_now, 100, 0, 0);
            Debugger::Armortrackbar(this->armor_detector, this->camera, window_name_target, window_name_proc);
        }
        imshow(window_name_target, image_draw);
        imshow(window_name_proc, armor_detector.processed_image);
        if (!target_armor.number_img.empty()) 
        {
            imshow(window_name_number, target_armor.number_img);
        }
    }
    else if (work_msg.mode == Mode::MODE_SMALLRUNE || work_msg.mode == Mode::MODE_BIGRUNE) 
    {
        // rune_detector.drawToDebug();
        // if (TRACKBAR) {
        //     cv::createTrackbar("video_progress", window_name_target, &video_now, 100, 0, 0);
        //     Debugger::Runetrackbar(this->rune_detector, this->camera, window_name_target, window_name_proc);
        // }
        cv::imshow(window_name_target, image_draw);
        if (!rune_detector.processed_image.empty())
        {
            cv::imshow(window_name_proc, rune_detector.processed_image);
        }
        // cv::waitKey(1);
    }
    else if (work_msg.mode == Mode::MODE_SENTINEL)
    {
        copyMakeBorder
        (
            sentinel_hunter.processed_image,
            sentinel_hunter.processed_image, 0,
            camera->frame_height - sentinel_hunter.processed_image.rows, 0,
            camera->frame_width - sentinel_hunter.processed_image.cols,
            cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255)
        );

        cv::imshow(window_name_target, image_draw);
        cv::imshow(window_name_proc, sentinel_hunter.processed_image);
    }
    // cv::resize(image_draw, image_draw, cv::Size(640, 5));
    
#ifdef GET_TRIGGER_IMAGE   
    static int value = this->soft_triger_synchronizor;
    cv::createTrackbar("synchronize", window_name_target, &value, 30, 0, 0);
    this->soft_triger_synchronizor = (value - 15);
#elif defined GET_CONSTANT_IMAGE
    // static int value = this->constant_synchronizor;
    // cv::createTrackbar("synchronize", window_name_target, &value, 30, 0, 0);
    // this->constant_synchronizor = (value - 15);
#endif

    static int armor_yaw = Util::yaw_skew_on_armor;
    cv::createTrackbar("armor_yaw", window_name_target, &armor_yaw, 100, 0, 0);
    Util::yaw_skew_on_armor = (armor_yaw-50)/10.0;

    static int armor_pitch = Util::pitch_skew_on_armor;
    cv::createTrackbar("armor_pitch", window_name_target, &armor_pitch, 200, 0, 0);
    Util::pitch_skew_on_armor = (armor_pitch-100)/10.0;

    // static int rune_yaw = Util::yaw_skew_on_rune;
    // cv::createTrackbar("rune_yaw", window_name_target, &rune_yaw, 100, 0, 0);
    // Util::yaw_skew_on_rune = (rune_yaw-50)/10.0;

    // static int rune_pitch = Util::pitch_skew_on_rune;
    // cv::createTrackbar("rune_pitch", window_name_target, &rune_pitch, 100, 0, 0);
    // Util::pitch_skew_on_rune = (rune_pitch-50)/10.0;

    // cv::imshow(window_name_target, image_draw);

    cv::waitKey(1);  //必须要有
}

void Workspace::recordVideo(cv::Mat &image)  // 记录图像的具体信息
{
    if (!writer.isOpened()) 
    {
        initVideoRecorder();
    }
    if (video_cnt > 15000) 
    {
        writer.release();
        video_cnt = 0;
        initVideoRecorder();
    } 
    else 
    {
        // 曝光模式
        int auto_exp = 0;
        CameraGetAeState(camera->hCamera, &auto_exp);
        cv::putText(image, "exp_mode: " + to_string(auto_exp), cv::Point(20, 160), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        // gamma值信息
        int gamma;
        CameraGetGamma(camera->hCamera, &gamma);
        cv::putText(image, "gamma: " + to_string(gamma), cv::Point(20, 180), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        // 对比度信息
        int contrast;
        CameraGetContrast(camera->hCamera, &contrast);
        cv::putText(image, "conntrast: " + to_string(contrast), cv::Point(20, 200), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        // 模拟增益值信息
        int analog_gain;
        CameraGetAnalogGain(camera->hCamera, &analog_gain);
        cv::putText(image, "analog_gain: " + to_string(analog_gain), cv::Point(20, 220), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        // 降噪使能信息
        int enableNoiseFilter;
        CameraGetNoiseFilterState(camera->hCamera, &enableNoiseFilter);
        cv::putText(image, " noise_filter: " + to_string(enableNoiseFilter), cv::Point(20, 240), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 255, 0));
        // 相机当前帧率模式信息
        int current_frame_speed_index;
        CameraGetFrameSpeed(camera->hCamera, &current_frame_speed_index);
        cv::putText(image, "frame_speed_index: " + to_string(current_frame_speed_index), cv::Point(20, 260), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
        double current_exposure_time;
        CameraGetExposureTime(camera->hCamera, &current_exposure_time);
        cv::putText(image, "exp_time: "+to_string(current_exposure_time), cv::Point(image.cols-200, 20), cv::QT_FONT_NORMAL, 0.6, cv::Scalar(0, 0, 255));
    }
    writer.write(image);
    video_cnt++;
}

void Workspace::initVideoRecorder() 
{
    // 基于当前系统的当前日期/时间
    time_t now = time(0);
    tm* ltm = localtime(&now);
    // 输出 tm 结构的各个组成部分
    string year = to_string(1900 + ltm->tm_year);
    string month = to_string(ltm->tm_mon + 1);
    string day = to_string(ltm->tm_mday);
    string hour = to_string(ltm->tm_hour);
    string minute = to_string(ltm->tm_min);
    string second = to_string(ltm->tm_sec);
    string name = VIDEO_SAVE_PATH + year + "-" + month + "-" + day + "-" + hour + ":" + minute + ":" + second + ".avi";
    writer.open(name, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 90, cv::Size(camera->frame_width, camera->frame_height), true);
    if (!writer.isOpened()) 
    {
        LOGE("Record video failed, couldn't init Recorder!");
        exit(1);  //比赛时保险起见注释
    }
}


void Workspace::compensateSkew(ReadPack &read_pack)
{
    if (read_pack.mode == Mode::MODE_ARMOR2 || read_pack.mode == Mode::MODE_SENTINEL)
    {
        read_pack.ptz_yaw += Util::yaw_skew_on_armor;
        read_pack.ptz_pitch += Util::pitch_skew_on_armor;
    } else {
        //  do thing
        // read_pack.ptz_yaw += Util::yaw_skew_on_rune;
        // read_pack.ptz_pitch += Util::pitch_skew_on_rune;
    }
}