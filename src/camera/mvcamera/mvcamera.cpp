#include "mvcamera.h"
#include "workspace.h"

using namespace cv;
using namespace std;

static void linear_predict_pose(Pose &result, Pose &pose1, Pose &pose2);

#ifdef SOFT_TRIGGER_MODE
    // 触发回调函数
    void GrabImageCallback(CameraHandle hCamera, unsigned char *pbyBuffer, tSdkFrameHead *p_sFrameInfo, PVOID pContext) 
    {
        // 首先进行强制转换
        MVCamera *p = (MVCamera *)pContext;

        CameraSdkStatus status;
        // 将 RAW 数据转换成指定格式的图像数据，默认转换为 BGR24 格式的图像数据。
        status = CameraImageProcess(hCamera, pbyBuffer, p->g_pRgbBuffer, p_sFrameInfo);

        // 使用原图像生成 OpenCV 中的 Mat
        if (status == CAMERA_STATUS_SUCCESS) 
        {
            // 此时说明拍照成功，且图片成功读到电脑缓存区 pbyBuffer 中，并且已经转换了格式，那么认为此时图片有效
            p->image_object.source_image = Mat(Size(p_sFrameInfo->iWidth, p_sFrameInfo->iHeight), CV_8UC3, p->g_pRgbBuffer);
            p->soft_trigger_image_valid = true;
        } else 
        {
            // 如果出现问题，认为该次触发时还没有接收到图片，标志位置 false
            p->soft_trigger_image_valid = false;
        } 
    }

#elif defined CONSTANT_MODE_HERO        // 连续取图模式下的回调函数
    void GrabImageCallback(CameraHandle hCamera, unsigned char *pbyBuffer,tSdkFrameHead *p_sFrameInfo, PVOID pContext) 
    {
        // 首先进行强制转换
        MVCamera *p = (MVCamera *)pContext;
        // 立刻计算回调刚获取的图像的时间
        p->image_object.pose.mcu_time = p->getShutterMCUTime(p->p_workspace->workspace_timer, p->p_workspace->constant_synchronizor);

        // 读图计时器
        Timer image_getting_timer;
        // 读图开始时刻
        double image_getting_start_time;

        if (p->p_workspace->RUNNING_TIME) 
        {
            image_getting_start_time = image_getting_timer.getTime();
        }

        // 获取该张图像
        if (CameraImageProcess(hCamera, pbyBuffer, p->g_pRgbBuffer, p_sFrameInfo) == CAMERA_STATUS_SUCCESS) 
        {
            p->image_object.source_image = Mat(Size(p_sFrameInfo->iWidth, p_sFrameInfo->iHeight), CV_8UC3, p->g_pRgbBuffer);
        } else {
            return;
        }
        // 计算该张图像的姿态
        // 首先判断姿态队列长度是否足以预测新的图像姿态
        if (p->p_workspace->pose_queue.length > 2) 
        {
            // 对姿态队列上锁，获取用来预测新图片姿态的姿态队列中两个最新姿态
            p->p_workspace->pose_queue_metux.lock();
            Pose pose1;
            p->p_workspace->pose_queue.back(pose1, 2);
            Pose pose2;
            p->p_workspace->pose_queue.back(pose2, 1);
            p->p_workspace->pose_queue_metux.unlock();
            // 保证姿态的实时性
            if (abs(pose2.mcu_time - p->p_workspace->workspace_timer.getMCUTime()) < 100) 
            {
                linear_predict_pose(p->image_object.pose, pose1, pose2);
                // 存入图像缓存区，先对图像缓存区进行上锁
                p->p_workspace->image_buffer_mutex.lock();
                // 如果缓存区过多，进行清空之后再存储
                if (p->p_workspace->image_object_buffer.size() > 50) 
                {
                    p->p_workspace->image_object_buffer.clear();
                }
                p->p_workspace->image_object_buffer.emplace_back(p->image_object);
                p->p_workspace->image_buffer_mutex.unlock();
            }
        }else 
        {
            LOGW("couldn't get pose data, thus can't get image");
        }

        if (p->p_workspace->SAVE_VIDEO == 1 && !p->image_object.source_image.empty())
        {
            p->p_workspace->recordVideo(p->image_object.source_image);
        }

        if (p->p_workspace->RUNNING_TIME) 
        {
            cout << "回调函数中读图所用时间："
                << image_getting_timer.getTime() - image_getting_start_time << "ms"
                << endl;
        }
        // cout << "发生回调" << endl;
    }
#elif defined CONSTANT_MODE_SJ
        void GrabImageCallback(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext)
        {
            MVCamera *p =  (MVCamera *)pContext;
            ImageClass tmp_image_object;
            // 立刻计算回调刚获取的图像的时间
            tmp_image_object.pose.mcu_time = p->getShutterMCUTime(p->p_workspace->workspace_timer, p->p_workspace->constant_synchronizor);
            // 读图计时器
            Timer image_getting_timer;
            // 读图开始时刻
            double image_getting_start_time;
            if (p->p_workspace->RUNNING_TIME) 
            {
                image_getting_start_time = image_getting_timer.getTime();
            }

            CameraSdkStatus status;
            status = CameraImageProcess(hCamera, pFrameBuffer, p->g_pRgbBuffer, pFrameHead);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                LOGE("func:CameraImageProcess failed!");
                return;
            } else {
                tmp_image_object.source_image = cv::Mat(pFrameHead->iHeight, pFrameHead->iWidth, CV_8UC3, p->g_pRgbBuffer).clone();
                /* 计算该张图像的姿态 */ 
                if (p->p_workspace->pose_queue.length > 2)  // 首先判断姿态队列长度是否足以预测新的图像姿态
                {
                    // 对姿态队列上锁，获取用来预测新图片姿态的姿态队列中两个最新姿态
                    p->p_workspace->pose_queue_metux.lock();
                    Pose pose1;
                    p->p_workspace->pose_queue.back(pose1, 2);
                    Pose pose2;
                    p->p_workspace->pose_queue.back(pose2, 1);
                    p->p_workspace->pose_queue_metux.unlock();
                    // 保证姿态的实时性
                    if (abs(pose2.mcu_time - p->p_workspace->workspace_timer.getMCUTime()) < 100) 
                    {
                        linear_predict_pose(tmp_image_object.pose, pose1, pose2);
                        p->image_object_buffer.push(tmp_image_object);
                    }else{
                        LOGW("pose data is outdated!");
                        return;
                    }
                }else {
                    LOGW("couldn't get pose data, thus can't push image_object to buffer!");
                    return;
                }
                if (p->p_workspace->SAVE_VIDEO == 1 && !tmp_image_object.source_image.empty())
                {
                    p->p_workspace->recordVideo(tmp_image_object.source_image);
                }
                if (p->p_workspace->RUNNING_TIME) 
                {
                    std::cout << "回调函数中读图所用时间："
                        << image_getting_timer.getTime() - image_getting_start_time << "ms"
                        << std::endl;
                }
            }
        }
        // TODO
#else 
    std::cout << "error get image mode!" << std::endl;
#endif

MVCamera::MVCamera(Workspace *p_workspace_) 
{
    is_open = false;
    p_workspace = p_workspace_;
}

MVCamera::~MVCamera() { close(); }

void MVCamera::init(const cv::FileStorage &file_storage) 
{
    cv::FileNode camera_set = file_storage["camera_setting"];
    camera_set["FRAME_WIDTH"] >> frame_width;
    camera_set["FRAME_HEIGHT"] >> frame_height;
    camera_set["ARMOR_EXPOSURE_TIME"] >> armor_exposure_time;
    camera_set["RUNE_EXPOSURE_TIME"] >> rune_exposure_time;
    camera_set["SENTINEL_EXPOSURE_TIME"] >> sentinel_exposure_time;
}

void MVCamera::open() 
{
    int status = -1;
    int channel = 3;
    // 设备的个数指针，调用时传入pCameraList数组的元素个数，函数返回时，保存实际找到的设备个数。
    // 注意，指向的值必须初始化，且不超过pCameraList数组元素个数，否则有可能造成内存溢出。
    int cameraCounts = 1;

    // 初始化SDK
    CameraSdkInit(0);
    // 枚举设备，并建立设备列表
    status = CameraEnumerateDevice(&tCameraEnumList, &cameraCounts);
    if (cameraCounts == 0) {
        throw CameraException("Enumerate camera failed.");
    } else if (cameraCounts >= 1) {
        printf("%d camera device detected!", cameraCounts);
    }

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    // hCamera
    // 相机句柄在调用其他相机相关的操作接口时，都需要传入该句柄，主要用于多相机之间的区分。
    status = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    if (status != CAMERA_STATUS_SUCCESS) {
        throw CameraException("Init camera failed.");
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    // 设定相机捕获图像的回调函数
    // 当捕获到新的图像数据帧时，GrabImageCallback(回调函数)就会被调用
    CameraSetCallbackFunction(hCamera, GrabImageCallback, (PVOID)this, NULL);

    // 为处理后图像输出的缓冲区开辟空间
    g_pRgbBuffer =
        (unsigned char *)malloc(tCapability.sResolutionRange.iWidthMax *
                                tCapability.sResolutionRange.iHeightMax * 3);

    /* 让 SDK 进入图像采集模式，开始接收来自相机发送的图像数据。
       如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像 */
    CameraPlay(hCamera);

#ifdef GET_TRIGGER_IMAGE
    /* 设置成软触发模式 */
    CameraSetTriggerMode(hCamera, 1);
    /* 设置一次触发只获得一帧图像 */
    CameraSetTriggerCount(hCamera, 1);

#elif defined GET_CONSTANT_IMAGE
    /* 设置成连续取图模式 */
    CameraSetTriggerMode(hCamera, 0);

#endif
    // 相机的分辨率描述
    tSdkImageResolution imageResolution;

    // TODO 不知道这个有啥用
    // 获得当前预览的分辨率。
    CameraGetImageResolution(hCamera, &imageResolution);

    imageResolution.iIndex = 0XFF;
    imageResolution.iWidth = frame_width;
    imageResolution.iHeight = frame_height;

    // 设置图像的分辨率
    CameraSetImageResolution(hCamera, &imageResolution);
    // 设置帧率
    CameraSetFrameSpeed(hCamera, current_frame_speed_index);
    // 设置gamma
    CameraSetGamma(hCamera, gamma);
    // 设置对比度
    CameraSetContrast(hCamera, contrast);
    // 设置饱和度,默认100
    CameraSetSaturation(hCamera, saturation);
    // 设置BRG通道增益
    CameraSetGain(hCamera, rchannel_gain, gchannel_gain, bchannel_gain);
    // 设置图像降噪使能 or not
    CameraSetNoiseFilter(hCamera, enableNoiseFilter);
    // 设置图像锐化
    CameraSetSharpness(hCamera, sharpness);

/**************设置曝光模式手动或者自动*******************/
#ifdef FIX_EXP
    //设置手动曝光
    CameraSetAeState(hCamera, false);
    // 设置曝光值
    CameraSetExposureTime(hCamera, armor_exposure_time);
    // 设置模拟增益
    CameraSetAnalogGain(hCamera, analog_gain);
#elif defined AUTO_EXP
    // 设置为自动曝光
    status = CameraSetAeState(hCamera, true);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("auto exp fail!");
        exit(1);
    }

    // 设置自动曝光目标亮度值
    status = CameraSetAeTarget(hCamera, auto_target_brightness);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set target brightness fail!");
        exit(1);
    }

    // 设置自动曝光的曝光范围,单位ms
    status = CameraSetAeExposureRange(hCamera, auto_min_expourse_time,
                                      auto_max_expourse_time);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set exp range fail!");
        exit(1);
    }

    // 设置自动曝光的增益范围
    status = CameraSetAeAnalogGainRange(hCamera, auto_min_analog_gain, auto_max_analog_gain);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set analog gain range fail!");
        exit(1);
    }

    // 设置自动曝光的阈值，abs(目标亮度-图像亮度) < iThreshold 则停止自动调节
    status = CameraSetAeThreshold(hCamera, auto_thresh);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set auto exp thresh fail!");
        exit(1);
    }

    // 设置自动曝光参考窗口,注意数据类型是int, 取中间的一块窗口
    status = CameraSetAeWindow(hCamera, 0, 0, 640, 480);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set refer window fail!");
        exit(1);
    }

    // 关闭抗频闪功能,官方的抗频闪实际上是通过调制曝光时间(10ms/20ms)
    status = CameraSetAntiFlick(hCamera, false);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set anti-flick fail!");
        exit(1);
    }

    // 设置自动曝光消频闪的频率，对于国内来是说因为50hz交流电
    // 0:50hz      1:60hz
    status = CameraSetLightFrequency(hCamera, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set anti-flick frequence fail!");
        exit(1);
    }
#else
    LOGE("didn't define camera exp mode!")
    exit(1);
#endif
    /*********************END******************************/

    // 判断相机的通道个数
    if (tCapability.sIspCapacity.bMonoSensor) {
        channel = 1;  // 8位单通道
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    } else {
        channel = 3;  // 8位三通道
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    is_open = true;

    // 输出相机工作信息，确定相机处于正常工作状态表
    camera_info_file.open("../logs/mvcamera_info.txt", ios::out);
    if (!camera_info_file.is_open()) {
        LOGW("camera file open error!");
    }
    getCameraInfo();
    camera_info_file.close();
}

bool MVCamera::isOpen() { return is_open; }

void MVCamera::getImage(Mat &image) 
{
    if (!is_open) {
        throw CameraException("Get image error. Camera is not opened.");
    }

    CameraGetExposureTime(hCamera, &exposure_time);
    camera_printf("相机曝光时间: %lf", exposure_time);
    CameraGetAnalogGain(hCamera, &analog_gain);
    camera_printf("相机模拟增益: %d", analog_gain);

    // 从相机的缓冲区中接受一幅图像，RAW图像，对于彩色相机是Bater格式的数据,最多等待500ms
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 2000) == CAMERA_STATUS_SUCCESS) 
    {
        // 将图像转换为RGB图像
        // 将获得的相机原始输出图像数据进行处理，叠加饱和度、
        // 颜色增益和校正、降噪等处理效果，最后得到RGB888格式的图像数据。
        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        // 使用原图像生成OpenCV中的Mat
        image = Mat(Size(sFrameInfo.iWidth, sFrameInfo.iHeight), CV_8UC3, g_pRgbBuffer);

        /* 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer
           否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，
           直到其他线程中调用CameraReleaseImageBuffer来释放了buffer */
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    } else {
        throw CameraException("Get image failed. Waiting time is too long.");
    }
}

int64_t MVCamera::getShutterMCUTime(Timer &workspace_timer, int64_t synchronizor) 
{
    // 记录当前的电控时刻
    int64_t current_mcu_time = workspace_timer.getMCUTime();
    // 当前曝光时间
    double current_exposure_time;
    CameraGetExposureTime(hCamera, &current_exposure_time);
    // 拍照时的电控时刻
#ifdef SOFT_TRIGGER_MODE
    int64_t shutter_mcu_time =  current_mcu_time +
                                static_cast<int64_t>(current_exposure_time / 2000.0) + 
                                synchronizor;  // TODO
#elif defined CONSTANT_MODE_HERO
    /* 连续取图模式下，回调函数是在最新拍摄的图片从相机缓存区拿到电脑缓存区之后进行调用，
    这个传输过程所用的时间用补偿量 synchronizor
    进行调整；因此真实的拍照时间是减去 */
    int64_t shutter_mcu_time =
        current_mcu_time -s
        static_cast<int64_t>(current_exposure_time / 2000.0) +
        synchronizor;  // TODO
#elif defined CONSTANT_MODE_SJ
    int64_t shutter_mcu_time =
        current_mcu_time -
        static_cast<int64_t>(current_exposure_time / 2000.0) +
        synchronizor;  // TODO
#endif
    return shutter_mcu_time;
}

void MVCamera::CameraSetForRune() 
{
    // 设置曝光模式为手动曝光
    CameraSetAeState(hCamera, false);
    // 设置曝光时间
    CameraSetExposureTime(hCamera, rune_exposure_time);
    // 设置模拟增益值
    CameraSetAnalogGain(hCamera, analog_gain);
}

void MVCamera::CameraSetForSentinel() {
    // 设置曝光模式为手动曝光
    CameraSetAeState(hCamera, false);
    // 设置曝光时间
    CameraSetExposureTime(hCamera, sentinel_exposure_time);
    // 设置模拟增益值
    CameraSetAnalogGain(hCamera, analog_gain);
}

void MVCamera::CameraSetForAromr() {
#ifdef USE_FIX_EXP
    //设置手动曝光
    CameraSetAeState(hCamera, false);
    // 设置曝光值
    CameraSetExposureTime(hCamera, armor_exposure_time);
    // 设置模拟增益
    CameraSetAnalogGain(hCamera, analog_gain);
#else
    // 设置为自动曝光
    int status = 0;
    status = CameraSetAeState(hCamera, true);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("auto exp fail!");
    }

    // 设置自动曝光目标亮度值
    status = CameraSetAeTarget(hCamera, auto_target_brightness);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set target brightness fail!");
    }

    // 设置自动曝光的曝光范围,单位ms
    status = CameraSetAeExposureRange(hCamera, auto_min_expourse_time,
                                      auto_max_expourse_time);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set exp range fail!");
    }

    // 设置自动曝光的增益范围
    status = CameraSetAeAnalogGainRange(hCamera, auto_min_analog_gain,
                                        auto_max_analog_gain);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set analog gain range fail!");
    }

    // 设置自动曝光的阈值，abs(目标亮度-图像亮度) < iThreshold 则停止自动调节
    status = CameraSetAeThreshold(hCamera, auto_thresh);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set auto exp thresh fail!");
    }

    // 设置自动曝光参考窗口,注意数据类型是int,取中间的一块窗口
    status = CameraSetAeWindow(hCamera, 0, 0, 640, 480);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set refer window fail!");
    }

    // 设置抗频闪功能使能
    status = CameraSetAntiFlick(hCamera, false);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set anti-flick fail!");
    }

    // 设置自动曝光消频闪的频率，对于国内来是说因为50hz交流电
    // 0:50hz      1:60hz
    status = CameraSetLightFrequency(hCamera, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
        LOGE("set anti-flick frequence fail!");
    }
#endif
}

bool MVCamera::soft_trigger() {
    // 必要保护
    if (!is_open) {
        throw CameraException("Get image error. Camera is not opened.");
    }
    // 软触发状态
    int status;
    // 为了避免意外取到相机缓存中的旧图片，在给触发指令前先清空了缓存
    CameraClearBuffer(hCamera);
    status = CameraSoftTrigger(hCamera);
    soft_trigger_image_valid = false;
    if (status != CAMERA_STATUS_SUCCESS){
        LOGE("soft triger failed");
        return false;
    }
    // cout << "trigger state:" << status << endl;
    // 设置图片有效标志位为无效，只有从相机缓存区将图片读到 image_object
    // 之后才会有效
    return true;
}

void MVCamera::getCameraInfo() {
    // 相机状态
    camera_info_file << "camera_status: " << is_open << endl << endl;
    // 帧率信息
    CameraGetFrameSpeed(hCamera, &current_frame_speed_index);
    camera_info_file << "current_frame_speed_index: "
                     << current_frame_speed_index << endl;
    // 分辨率信息
    tSdkImageResolution imageResolution;
    CameraGetImageResolution(hCamera, &imageResolution);
    camera_info_file << "resolution: (" << imageResolution.iWidth << ", "
                     << imageResolution.iHeight << ")" << endl;

    // 曝光模式
    int auto_exp = 0;
    CameraGetAeState(hCamera, &auto_exp);
    camera_info_file << "auto exp status: " << auto_exp << endl;
    // 自动曝光目标亮度值
    CameraGetAeTarget(hCamera, &auto_target_brightness);
    camera_info_file << "auto exp aim brightness: " << auto_target_brightness
                     << endl;
    // 自动曝光阈值
    CameraGetAeThreshold(hCamera, &auto_thresh);
    camera_info_file << "auto exp thresh: " << auto_thresh << endl;
    // 自动曝光曝光范围
    CameraGetAeExposureRange(hCamera, &auto_min_expourse_time,
                             &auto_max_expourse_time);
    camera_info_file << "auto exp time range: "
                     << "(" << auto_min_expourse_time << ","
                     << auto_max_expourse_time << ")" << std::endl;
    // 自动曝光相机模拟增益
    CameraGetAeAnalogGainRange(hCamera, &auto_min_analog_gain,
                               &auto_max_analog_gain);
    camera_info_file << "auto exp analog gain: "
                     << "(" << auto_min_analog_gain << ","
                     << auto_max_analog_gain << ")" << std::endl;

    // gamma值信息
    CameraGetGamma(hCamera, &gamma);
    camera_info_file << "gamma: " << gamma << endl;
    // 对比度信息
    CameraGetContrast(hCamera, &contrast);
    camera_info_file << "constrast: " << contrast << endl;
    // 模拟增益值信息
    CameraGetAnalogGain(hCamera, &analog_gain);
    camera_info_file << "fix exp analog gain: " << analog_gain << endl;
    // 降噪使能信息
    CameraGetNoiseFilterState(hCamera, &enableNoiseFilter);
    camera_info_file << "enableNoiseFilter status: " << enableNoiseFilter
                     << endl;
    // 图像锐化信息
    CameraGetSharpness(hCamera, &sharpness);
    camera_info_file << "sharpness: " << sharpness << std::endl;

    // 相机支持的帧率模式信息
    cout << endl;
    camera_info_file << "number of frame speed models: "
                     << tCapability.iFrameSpeedDesc << endl;
    // 相机当前帧率模式信息
    int current_frame_speed_index;
    CameraGetFrameSpeed(hCamera, &current_frame_speed_index);
    camera_info_file << "current frame speed model index: "
                     << current_frame_speed_index << endl
                     << endl;
}

void MVCamera::close() {
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
    is_open = false;
}

// 线性姿态预测函数
static void linear_predict_pose(Pose &result, Pose &pose1, Pose &pose2) 
{
    // 以 pose1.mcu_time 为时间原点
    // 计算各姿态分量的增量
    double delta_ptz_pitch = pose2.ptz_pitch - pose1.ptz_pitch;
    double delta_ptz_yaw = pose2.ptz_yaw - pose1.ptz_yaw;
    double delta_ptz_roll = pose2.ptz_roll - pose1.ptz_roll;
    // 计算 ptz-t 直线的斜率
    int64_t delta_t = (pose2.mcu_time - pose1.mcu_time);
    double k_ptz_pitch = 1.0 * delta_ptz_pitch / delta_t;
    double k_ptz_yaw = 1.0 * delta_ptz_yaw / delta_t;
    double k_ptz_roll = 1.0 * delta_ptz_roll / delta_t;
    // 得到直线即为 result.ptz = k * result.mcu_time + pose1.ptz

    result.ptz_pitch =
        k_ptz_pitch * (result.mcu_time - pose1.mcu_time) + pose1.ptz_pitch;

    result.ptz_yaw =
        k_ptz_yaw * (result.mcu_time - pose1.mcu_time) + pose1.ptz_yaw;
        
    result.ptz_roll =
        k_ptz_roll * (result.mcu_time - pose1.mcu_time) + pose1.ptz_roll;
}