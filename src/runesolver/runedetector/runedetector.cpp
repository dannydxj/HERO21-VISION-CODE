#include <math.h>
#include <opencv2/opencv.hpp>

#include "runedetector.h"
void RuneDetector::init(const cv::FileStorage &file_storage) 
{
    loadParam(file_storage);
    // 输出信息
}

bool RuneDetector::run(ImageClass &image_object, const ReadPack &mcu_data) 
{
    onEnter();
    if (!mcu_data.isRightMouseButtonClicked && (flag_start==0)) /* 无触发且线程不工作 */
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 睡眠1ms
        return false;
    } else if (!mcu_data.isRightMouseButtonClicked && (flag_start == 1))
    { /* 无触发且线程工作 */
        // do thing
    }
    else if(mcu_data.isRightMouseButtonClicked)
    { /* 有触发，清空重置*/
        /* 记录一下当前的yaw和pitch */
        clear();
        central_yaw = image_object.pose.ptz_yaw;
        energy_yaw = image_object.pose.ptz_yaw;
        central_pitch = image_object.pose.ptz_pitch;
        energy_yaw += 90.0; // 很坑
        /* flag_start 置1 */
        flag_start = 1;
    }
    /* flag_start==1 的时候执行 */
    processImage(image_object.source_image, mcu_data.enemy_color);
    if (findTarget(processed_image, todo_candidate_rects, done_candidate_rects))
    {
        drawToDebug();
        return true;
    } else
    {
        return false;
    }
}

void RuneDetector::processImage(cv::Mat &origin_image, int color) 
{
    cv::Mat dilate_img;
    cv::Mat open_img;
    cv::Mat close_img;
    std::vector<cv::Mat> channels;
    debug_image = origin_image;
    origin_image.copyTo(src_image);

    /// 能量机关需要做一下颜色转换
    if (color == EnemyColor::COLOR_RED){
        color = EnemyColor::COLOR_BLUE;
    }else{
        color = EnemyColor::COLOR_RED;
    }

    // 选择二值化原图（1. 灰度图 2. 颜色通道相减图 3. 单颜色通道图 & 灰度图）
    // 对于颜色比较清楚而且背景有白光的首选2 、3，比赛时可以通过固定低曝光值实现
    // 对于曝光比较高的选择1，但是1是没有办法排除白光的干扰
#ifdef RUNE_GRAY
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
    threshold(gray_image, binary_image, RUNE_GRAY_THRES, 255, cv::THRESH_BINARY);
    // 膨胀操作目的是让箭头连在一块, 断开的灯条连在一起，
    dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1));
    cv::dilate(binary_image, dilate_img, dilate_kernel);
    // 闭运算，目的是填补白色区域中可能出现的小洞，避免其干扰轮廓查找
    close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * close_kernel_size + 1, 2 * close_kernel_size + 1));
    cv::morphologyEx(dilate_img, close_img, cv::MORPH_CLOSE, close_kernel);
    processed_image = close_img.clone();
#elif defined RUNE_SUBTRACT
    split(src_image, channels);
    if (color == COLOR_BLUE) {
        threshold(channels.at(0) - channels.at(2), binary_image,
                  RUNE_BLUE_THRESH, 255, cv::THRESH_BINARY);
    } else {
        threshold(channels.at(2) - channels.at(0), binary_image,
                  RUNE_RED_THRESH, 255, cv::THRESH_BINARY);
    }
    // 开运算去除噪点
    open_kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * open_kernel_size + 1, 2 * open_kernel_size + 1));
    cv::morphologyEx(binary_image, open_img, cv::MORPH_CLOSE, open_kernel);
    processed_image = open_img.clone();
#elif defined RUNE_ARMOR_METHOD
    cv::Mat channel_img;
    cv::Mat temp_binary_img;
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
    threshold(gray_image, temp_binary_img, RUNE_GRAY_THRES - 40, 255, cv::THRESH_BINARY);
    split(src_image, channels);
    if (color == COLOR_BLUE) {
        threshold(channels[0], channel_img, RUNE_BLUE_CHANNEL_THRESH, 255, cv::THRESH_BINARY);
    } else {
        threshold(channels[2], channel_img, RUNE_RED_CHANNEL_THRESH, 255, cv::THRESH_BINARY);
    }
    channel_img = channel_img & temp_binary_img;
    // 膨胀操作目的是让箭头连在一块, 断开的灯条连在一起
    dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1));
    cv::dilate(channel_img, dilate_img, dilate_kernel);
    processed_image = dilate_img;
#elif defined RUNE_RANGE_THRESH
    Timer timer;
    threshImage(src_image, color);
    timer.printTime("算法速度");
#else
    std::cout << "rune wrong preprocess mode!" << std::endl;
    exit(1);
#endif
}

bool RuneDetector::findTarget(cv::Mat &processed_image,
                            std::vector<Candidate_Rect> &todo_candidate_rects,
                            std::vector<Candidate_Rect> &done_candidate_rects) 
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(processed_image, contours, hierarchy, CV_RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours.size(); i++) 
    {
        // 寻找父轮廓, 没有父轮廓或者轮廓点数太少丢掉,
        // 拟合椭圆至少需要5个点，但一般来说需要6个点拟合
        if (hierarchy[i][3] < 0 || contours[i].size() < 6 ||
            contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        rune_printf("111");
        // 对轮廓进行椭圆的矩形拟合
        cv::RotatedRect ellipse_child_rect = cv::fitEllipse(contours[i]);
        cv::RotatedRect ellipse_dad_rect =
            cv::fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);

        // 用角度差筛选，理想情况下角度差为90
        double diff_angle =
            fabsf(ellipse_child_rect.angle - ellipse_dad_rect.angle);
        if (diff_angle > 100 || diff_angle < 80) continue;
        rune_printf("222");

        // 使用装甲板长短比进一步筛选
        double child_ellipse_ratio =
            ellipse_child_rect.size.height / ellipse_child_rect.size.width;
        if (child_ellipse_ratio < 1 || child_ellipse_ratio > 3) continue;
        rune_printf("333");

        // 用面积和边长进一步筛选子轮廓
        double child_contour_area = cv::contourArea(contours[i]);
        double child_contour_length = cv::arcLength(contours[i], true);
        if (child_contour_area < RUNE_MIN_CHILD_CONTOUR_AREA ||
            child_contour_area > RUNE_MAX_CHILD_CONTOUR_AREA ||
            child_contour_length < RUNE_MIN_CHILD_CONTOUR_LENGTH ||
            child_contour_length > RUNE_MAX_CHILD_CONTOUR_LENGTH)
            continue;
        rune_printf("444");

        // 用面积和边长进一步筛选父轮廓
        double dad_contour_area =
            cv::contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double dad_contour_length =
            cv::arcLength(contours[static_cast<uint>(hierarchy[i][3])], true);
        if (dad_contour_area < RUNE_MIN_DAD_CONTOUR_AREA ||
            dad_contour_area > RUNE_MAX_DAD_CONTOUR_AREA ||
            dad_contour_length < RUNE_MIN_DAD_CONTOUR_LENGTH ||
            dad_contour_length > RUNE_MAX_DAD_CONTOUR_LENGTH)
            continue;
        rune_printf("555");

        Candidate_Rect candidate_rect;

        // 使用子父轮廓面积之比来确定子轮廓装甲板类型（待激活、已激活）
        // 之所以把矩形放进判断里面是分类条件同时也是判断条件，矩形用MinAreaRect提取更加准确
        if (child_contour_area * 12 > dad_contour_area &&
            child_contour_area * 5 < dad_contour_area) {
            candidate_rect.isActivated = true;
            candidate_rect.rect = cv::minAreaRect(contours[i]);
            LOGM("已激活装甲板面积：%f\n", candidate_rect.rect.size.area());
            done_candidate_rects.emplace_back(candidate_rect);
        } else if (child_contour_area * 5 > dad_contour_area &&
                   child_contour_area * 2 < dad_contour_area) {
            candidate_rect.isActivated = false;  // 未激活
            candidate_rect.rect = cv::minAreaRect(contours[i]);
            LOGM("未激活装甲板面积：%f\n", candidate_rect.rect.size.area());
            todo_candidate_rects.emplace_back(candidate_rect);
        }
    }
    if (todo_candidate_rects.empty()) 
    {
        LOGW("未发现目标！");
        fail_cnt++;
        if (fail_cnt > 10)
        {
            fail_cnt = 10; // 不能超过3
            flag_center_aligning = true; // 失败次数超过阈值，执行对心操作
        }
        return false;
    } 
    else if (todo_candidate_rects.size() != 1) 
    {
        // 如果能量机关在激活后的一瞬间存在不同步的时候也会出现这个问题
        LOGW("发现多个目标，疑似出现错误！");
        return false;
    } 
    else if (done_candidate_rects.size() >= 6) 
    {
        LOGW("已激活扇叶数量错误！");
        return false;
    } 
    else 
    {
        if (fail_cnt != 0) // TODO 需不需要这么做
        {
            fail_cnt -= 3;
            if (fail_cnt < 0)
            {
                fail_cnt = 0;
            }
            return false;
        } else 
        {
            flag_center_aligning = false; // 识别到装甲板，取消对心操作
            // 融合两个vector，第一个装甲板为待激活装甲板
            todo_candidate_rects.insert(todo_candidate_rects.end(), done_candidate_rects.begin(), done_candidate_rects.end());
            return true;
        }
    }
}



void RuneDetector::threshImage(cv::Mat &image, int color) 
{
    binary_image = cv::Mat::zeros(image.size(), CV_8UC1);  
    uchar *pdata = (uchar *)image.data;
    uchar *qdata = (uchar *)binary_image.data;
    int srcData = image.rows * image.cols;

    if (color == EnemyColor::COLOR_RED) {
        for (int i = 0; i < srcData; i++) 
        {
            int gray = (*pdata) * 0.114 + *(pdata + 1) * 0.587 + *(pdata + 2) * 0.299;
            if (*(pdata + 2) > RUNE_RED_CHANNEL_THRESH && gray < RUNE_GRAY_THRES)
            {
                *qdata = 255;
            }    
            pdata += 3;
            qdata++;
        }
    } else if (color == EnemyColor::COLOR_BLUE) 
    {
        for (int i = 0; i < srcData; i++) 
        {
            int gray = (*pdata) * 0.114 + *(pdata + 1) * 0.587 + *(pdata + 2) * 0.299;
            if (*pdata > RUNE_BLUE_CHANNEL_THRESH && gray < RUNE_GRAY_THRES)
            {
                *qdata = 255;
            }
            pdata += 3;
            qdata++;
        }
    }

    dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_kernel_size + 1, 2 * dilate_kernel_size + 1));
    cv::dilate(binary_image, processed_image, dilate_kernel);
}

void RuneDetector::onEnter() 
{
    todo_candidate_rects.clear();
    done_candidate_rects.clear();
}

void RuneDetector::drawToDebug() 
{
    if (todo_candidate_rects.empty())
    {
        return;
    } 
    // 绘制圆
    // cv::circle(debug_image, img_c, img_radius, cv::Scalar(0, 255, 0), 1, 8, 0);
    // 绘制识别的装甲板
    for (auto iter = todo_candidate_rects.begin(); iter != todo_candidate_rects.end(); ++iter) 
    {
        Util::drawRotatedRect(debug_image, iter->rect, cv::Scalar(0, 255, 0));
        if (iter->isActivated) 
        {
            cv::putText(debug_image, "done", (iter->rect).center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8);
        } else 
        {
            cv::putText(debug_image, "todo", (iter->rect).center, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, 8);
        }
    }
}


void RuneDetector::clear()
{
    flag_start = 0;
    energy_yaw = central_pitch = central_yaw = 0.0;
    fail_cnt = 0;
    flag_center_aligning = false;
}

void RuneDetector::loadParam(const cv::FileStorage &file_storage) 
{
    cv::FileNode rune_solve = file_storage["rune"];
    rune_solve["RUNE_GRAY_THRES"] >> RUNE_GRAY_THRES;
    rune_solve["RUNE_BLUE_THRESH"] >> RUNE_BLUE_THRESH;
    rune_solve["RUNE_RED_THRESH"] >> RUNE_RED_THRESH;
    rune_solve["RUNE_BLUE_CHANNEL_THRESH"] >> RUNE_BLUE_CHANNEL_THRESH;
    rune_solve["RUNE_RED_CHANNEL_THRESH"] >> RUNE_RED_CHANNEL_THRESH;
    rune_solve["RUNE_MIN_CHILD_CONTOUR_AREA"] >> RUNE_MIN_CHILD_CONTOUR_AREA;
    rune_solve["RUNE_MAX_CHILD_CONTOUR_AREA"] >> RUNE_MAX_CHILD_CONTOUR_AREA;
    rune_solve["RUNE_MIN_DAD_CONTOUR_AREA"] >> RUNE_MIN_DAD_CONTOUR_AREA;
    rune_solve["RUNE_MAX_DAD_CONTOUR_AREA"] >> RUNE_MAX_DAD_CONTOUR_AREA;
    rune_solve["RUNE_MIN_CHILD_CONTOUR_LENGTH"] >> RUNE_MIN_CHILD_CONTOUR_LENGTH;
    rune_solve["RUNE_MAX_CHILD_CONTOUR_LENGTH"] >> RUNE_MAX_CHILD_CONTOUR_LENGTH;
    rune_solve["RUNE_MIN_DAD_CONTOUR_LENGTH"] >> RUNE_MIN_DAD_CONTOUR_LENGTH;
    rune_solve["RUNE_MAX_DAD_CONTOUR_LENGTH"] >> RUNE_MAX_DAD_CONTOUR_LENGTH;
}