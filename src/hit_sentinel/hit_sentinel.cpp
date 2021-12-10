#include "hit_sentinel.h"

#include "macro_switch.h"

static void adjustLightBarRect(cv::RotatedRect &rect);
static void adjustLightBarAngle(float elli_angle, double &bar_angle);
static void judgeLightbarColor(Lightbar &lightbar, const cv::Mat &roi_image);

bool Sentinel::run(ImageClass &image_object, const int color) 
{
    onEnter();
    processImg(image_object.source_image, color);              /* 图像预处理 */
    findLightbars(color);        /* 寻找灯条 */
    matchArmors(lightbars, armors); /* 匹配装甲板 */
    drawBars(debug_image, lightbars);
    drawArmors(debug_image, armors);
    solve3Dcoordinates(armors, image_object); /* 转换到大地坐标系，原点为yaw和pitch转轴连线交点 */
    filterTarget(targets); /* 用距离过滤误识别装甲板, 分析录像的时候要将此函数注释掉 */
    selectTarget(targets);              /* 选出待打击的装甲板 */
    rectifyCoordinate(targets, STANDARD_HEIGTH); /* 使用标准高度距离进行坐标矫正 */
    // drawBars(debug_image, lightbars);
    // drawArmors(debug_image, armors);
    onExit();
    return target_found;
}

void Sentinel::init(const cv::FileStorage &file_storage) 
{
    roi_rect = cv::Rect();
    loadParam(file_storage);
}

void Sentinel::processImg(cv::Mat &image, const int color) 
{
    debug_image = image;
    if (!roi_rect.empty() && ROI_ENABLE) 
    {
        image(roi_rect).copyTo(roi_image);
    } else {
        image.copyTo(roi_image);
    }
    /* 转灰度图 */
    cv::cvtColor(roi_image, gray_image, cv::COLOR_BGR2GRAY);

    /* 转二值图 */
    if (color == EnemyColor::COLOR_RED)    
    {
        cv::threshold(gray_image, binary_image, RED_GRAY2BINARY_THRESH, 255, cv::THRESH_BINARY); 
    }else{
        cv::threshold(gray_image, binary_image, BLUE_GRAY2BINARY_THRESH, 255, cv::THRESH_BINARY); 
    }
#ifdef SENTINEL_OLD
    binary_image.copyTo(processed_image);
#elif defined SENTINEL_NEW
    /* bgr通道相减图 */
    std::vector<cv::Mat> channels;
    split(roi_image, channels);
    if (color == EnemyColor::COLOR_BLUE) 
    {
        subtract(channels[0], channels[2], subtract_image);
        threshold(subtract_image, subtract_image, BLUE_SUBTRACT_THRESH, 255, cv::THRESH_BINARY);
    } else 
    {
        subtract(channels[2], channels[0], subtract_image);
        threshold(subtract_image, subtract_image, RED_SUBTRACT_THRESH, 255, cv::THRESH_BINARY);
    }
    processed_image = binary_image & subtract_image;
#endif
}

void Sentinel::findLightbars(const int color) 
{
    // 找出所有轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    if (ROI_ENABLE && !roi_rect.empty())
    {
        findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(roi_rect.x, roi_rect.y));  // 寻找最外层轮廓
    }else{
        findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // 寻找最外层轮廓
    }
    for (auto &contour : contours) 
    {
        if (contour.size() < 6) 
            continue;

        Lightbar lightbar;
        lightbar.rect = cv::minAreaRect(contour);
        lightbar.elli_rect = cv::fitEllipse(contour);

        // 面积筛选
        if (lightbar.rect.size.area() < 100 && lightbar.rect.size.area() > 1) 
        {
            //矫正灯条minAreaRect，保证 height > width
            adjustLightBarRect(lightbar.rect);
            adjustLightBarAngle(lightbar.elli_rect.angle, lightbar.angle);
            // 角度、颜色筛选灯条
            if (abs(lightbar.angle) < 10) 
            {
                judgeLightbarColor(lightbar, debug_image);
                if (lightbar.color == color) 
                {
                    lightbars.emplace_back(lightbar);
                }
            }
        }
    }
    /* 候选灯条排序 */
    sort(lightbars.begin(), lightbars.end(), Lightbar::coordinateComparator);
}

void Sentinel::matchArmors(std::vector<Lightbar> &lightbars,
                           std::vector<Armor> &armors) 
{
    if (lightbars.size() <= 1) 
        return;

    for (auto lbar = lightbars.begin(); lbar != lightbars.end(); ++lbar) 
    {
        auto rbar = lbar;
        for (++rbar; rbar != lightbars.end(); ++rbar) 
        {
            // 装甲板比例，面积，灯条高度比例，灯条角度差
            //------------------像素距离-----------------------------------
            float delta_width = fabs(rbar->rect.center.x - lbar->rect.center.x);
            float delta_height = fabs(rbar->rect.center.y - lbar->rect.center.y);
            // 装甲板x距离过近
            if (delta_width < lbar->rect.size.height) 
                continue;
            sentinel_printf("111");
            // 装甲板x距离过远
            if (delta_width > lbar->rect.size.height * (MAX_ARMOR_RATIO + 1))
                break;
            sentinel_printf("222");
            // 装甲板y距离过远
            if (delta_height > 150.0) 
                continue;
            sentinel_printf("333");

            //---------------装甲板比例--------------------------------------
            double armor_width =
                sqrt(delta_width * delta_width + delta_height * delta_height);
            double armor_height =
                (lbar->rect.size.height + rbar->rect.size.height) / 2;
            double armor_ratio = armor_width / armor_height;
            if (armor_ratio < MIN_ARMOR_RATIO || armor_ratio > MAX_ARMOR_RATIO)
                continue;
            sentinel_printf("444");

            //--------------两灯条长度比-----------------------------------------
            double lbar_len_ratio = lbar->rect.size.height / rbar->rect.size.height;
            lbar_len_ratio = ((lbar_len_ratio > 1.0) ? lbar_len_ratio
                                                     : (1.0 / lbar_len_ratio));
            if (lbar_len_ratio > MAX_BAR_HEIGHT_RATIO) 
                continue;
            sentinel_printf("666");

            //-------------两灯条角度差-------------------------------------------
            double lbar_delta_angle = abs(lbar->angle - rbar->angle);
            if (lbar_delta_angle > MAX_BAR_DELTA_ANGLE) 
                continue;
            sentinel_printf("777");

            Armor armor;
            armor.isBigArmor = true;
            armor.assignMemberVariables(*lbar, *rbar);
            armors.emplace_back(armor);
        }
    }
}

bool Sentinel::selectTarget(std::vector<Target> &targets) 
{
    if (targets.size() < 1) 
    {
        LOGW("didn't find sentinel!");
        target_found = false;
        return false;
    } 
    else if (targets.size() > 1) 
    {
        std::sort(targets.begin(), targets.end(), [this](const Target &t1, const Target &t2)
                       {
                           double delta_z1 = fabs(t1.z - this->STANDARD_HEIGTH);
                           double delta_z2 = fabs(t2.z - this->STANDARD_HEIGTH);
                           return (delta_z1 < delta_z2);
                        });
                    
        LOGW("found mutil sentinels");
        ptz_target = targets.at(0);
        target_found = true;
        return true;
    } else {
        // LOGM("sentinel found");
        ptz_target = targets.at(0);
        target_found = true;
        return true;
    }
}

void Sentinel::filterTarget(std::vector<Target> &targets) 
{
    if (targets.empty()) 
        return;

    for (auto iter = targets.begin(); iter != targets.end();) 
    {
        double depth = iter->y;
        double height = iter->z;
        if (ifHeightSatisfied(height))  // 对深度上先不做限制，只对高度上做限制
        {
            ++iter;
        } else 
        {
            iter = targets.erase(iter);
        }
    }
}

void Sentinel::solve3Dcoordinates(std::vector<Armor> &armors, const ImageClass &image_object) 
{
    if (armors.empty())
    {
        return;
    } 
    for (auto iter = armors.begin(); iter != armors.end(); ++iter) 
    {
        cv::Point2f left_up, left_down, right_up, right_down;
        cv::Point2f left_vertices[4];
        cv::Point2f right_vertices[4];
        iter->lbar_rect.points(left_vertices);
        iter->rbar_rect.points(right_vertices);

        /* 按照y坐标进行排序。注意：y轴正方向向下
        将四个点的y坐标由小到大排序 */
        sort(left_vertices, left_vertices + 4,
             [](const cv::Point2f &p1, const cv::Point2f &p2) {
                 return p1.y < p2.y;
             });
        // 再根据四个点的y坐标确定相关位置
        left_up = (left_vertices[0] + left_vertices[1]) * 0.5;
        left_down = (left_vertices[2] + left_vertices[3]) * 0.5;
        sort(right_vertices, right_vertices + 4,
             [](const cv::Point2f &p1, const cv::Point2f &p2) {
                 return p1.y < p2.y;
             });

        right_up = (right_vertices[0] + right_vertices[1]) * 0.5;
        right_down = (right_vertices[2] + right_vertices[3]) * 0.5;

        double half_w = 112.5;
        double half_h = 27.50;

        /* 将装甲板中心作为空间坐标系的原点，
        其四个顶点作为2D-3D的四对点 */
        // points3d中的点需和points2d中的点按顺序一一对应
        static std::vector<cv::Point3f> points3d;
        points3d.clear();
        points3d.emplace_back(cv::Point3f(-half_w, -half_h, 0));
        points3d.emplace_back(cv::Point3f(half_w, -half_h, 0));
        points3d.emplace_back(cv::Point3f(half_w, half_h, 0));
        points3d.emplace_back(cv::Point3f(-half_w, half_h, 0));

        std::vector<cv::Point2f> points2d;
        points2d.clear();
        points2d.emplace_back(left_up);
        points2d.emplace_back(right_up);
        points2d.emplace_back(right_down);
        points2d.emplace_back(left_down);

        // 解算相机坐标系下的位置
        cv::solvePnP(points3d, points2d, CAMERA_MATRIX, DISTORTION_COEFF, rotate_mat, trans_mat, false, cv::SOLVEPNP_ITERATIVE);
        Target target;
        // 平移至yaw和pitch轴中心
        target.x = (trans_mat.at<double>(0, 0) + Util::X_OFFSET) / 1000;
        target.y = (trans_mat.at<double>(1, 0) + Util::Y_OFFSET) / 1000;
        target.z = (trans_mat.at<double>(2, 0) + Util::Z_OFFSET) / 1000;
        // 转换到大地坐标系
        Util::wheel2land(target, image_object);
        targets.emplace_back(target);
        iter->sum_error_score = target.x; // 暂时的一个非常狗屎的方法用于标志target和armor的关系
    }
}

void Sentinel::rectifyCoordinate(std::vector<Target> &targets, double standard_height) 
{
    if (targets.empty())
        return;
    for (auto &target : targets) 
    {
        k = static_cast<double>(standard_height / target.z);
        target.x *= k;
        target.y *= k;
        target.z *= k;
    }
}

bool Sentinel::ifDepthSatisfied(double armor_distance) 
{
    if (armor_distance < MAX_HEIGHT && armor_distance > MIN_DEPTH)
    {
        return true;
    } else 
    {
        return false;
    }
}

bool Sentinel::ifHeightSatisfied(double armor_height) 
{
    if (armor_height < MAX_HEIGHT && armor_height > MIN_HEIGHT) 
    {
        return true;
    } else {
        return false;
    }
}

void Sentinel::onEnter() 
{
    targets.clear();
    armors.clear();
    lightbars.clear();
}

void Sentinel::onExit() 
{
    if (ROI_ENABLE && !targets.empty()) 
    {
        for (auto &armor : armors)
        {
            if (fabs(armor.sum_error_score - (ptz_target.x/k)) < 0.01)
            {
                setRoiRect(armor.rect());
                break;
                // LOGM("面积：%lf", armor.armor_rect.size.area());
            }
        }
    }else 
    {
        roi_rect = cv::Rect();
    }
}

static void adjustLightBarRect(cv::RotatedRect &rect) 
{
    if (rect.size.width > rect.size.height) 
    {
        rect = cv::RotatedRect(rect.center, cv::Size2f(rect.size.height, rect.size.width), rect.angle + 90);
    }
}

static void adjustLightBarAngle(float elli_angle, double &bar_angle) 
{
    if (elli_angle > 90.0) 
    {
        bar_angle = elli_angle - 180.0;
    } else {
        bar_angle = elli_angle;
    }
}

static void judgeLightbarColor(Lightbar &lightbar, const cv::Mat &src) 
{
    auto region = lightbar.rect.boundingRect();

    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);

    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);

    int red_cnt = 0, blue_cnt = 0;

    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }

    if (red_cnt > blue_cnt) {
        lightbar.color = EnemyColor::COLOR_RED;
    } else {
        lightbar.color = EnemyColor::COLOR_BLUE;
    }
}

void Sentinel::setRoiRect(const cv::Rect &roiRect) 
{
    int detect_x, detect_y;
    int detect_width, detect_height;

    detect_width = static_cast<int>(roiRect.width * 2.0);
    detect_height = static_cast<int>(roiRect.height * 2.0);
    detect_x = roiRect.x - (detect_width - roiRect.width) / 2;
    detect_y = roiRect.y - (detect_height - roiRect.height) / 2;

    preventROIExceed(detect_x, detect_y, detect_width, detect_height);
    roi_rect = cv::Rect(detect_x, detect_y, detect_width, detect_height);
}

void Sentinel::preventROIExceed(int &x, int &y, int &width, int &height) 
{
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x + width > debug_image.cols)
        width = debug_image.cols;
    if (y + height > debug_image.rows)
        height = debug_image.rows - y;
}


void Sentinel::drawBars(cv::Mat &debug_image, std::vector<Lightbar> &bars) {
    LOGM("灯条数量%d", lightbars.size());
    for (auto &bar : bars) 
    {
        cv::circle(debug_image, bar.rect.center, 2, cv::Scalar(0, 255, 0), -1, 8, 0);
    }
}

void Sentinel::drawArmors(cv::Mat &debug_image, std::vector<Armor> &armors) {
    LOGM("装甲板数量%d", armors.size());
    if (armors.empty()) 
        return;
    for (auto &armor:armors)
    {
        Util::drawRotatedRect(debug_image, armor.armor_rect, cv::Scalar(0, 255, 0)); /* 绿色: 绘制待打击装甲板 */
    }
}


void Sentinel::loadParam(const cv::FileStorage &file_storage) 
{
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    file_storage["distortion_coeff"] >> DISTORTION_COEFF;

    cv::FileNode sentinel_detect = file_storage["sentinel"];
    sentinel_detect["SENTINEL_STANDARD_HEIGHT"] >> STANDARD_HEIGTH;
    sentinel_detect["SENTINEL_MIN_HEIGHT"] >> MIN_HEIGHT;
    sentinel_detect["SENTINEL_MAX_HEIGHT"] >> MAX_HEIGHT;
    sentinel_detect["SENTINEL_MIN_DEPTH"] >> MIN_DEPTH;
    sentinel_detect["SENTINEL_MAX_DEPTH"] >> MAX_DEPTH;
    sentinel_detect["SENTINEL_MIN_ARMOR_RATIO"] >> MIN_ARMOR_RATIO;
    sentinel_detect["SENTINEL_MAX_ARMOR_RATIO"] >> MAX_ARMOR_RATIO;
    sentinel_detect["SENTINEL_MIN_ARMOR_AREA"] >> MIN_ARMOR_AREA;
    sentinel_detect["SENTINEL_MAX_ARMOR_AREA"] >> MAX_ARMOR_AREA;
    sentinel_detect["SENTINEL_MAX_BAR_HEIGHT_RATIO"] >> MAX_BAR_HEIGHT_RATIO;
    sentinel_detect["SENTINEL_MAX_BAR_ANGLE"] >> MAX_BAR_DELTA_ANGLE;

    sentinel_detect["SENTINEL_RED_GRAY2BINARY_THRESH"] >> RED_GRAY2BINARY_THRESH;
    sentinel_detect["SENTINEL_BLUE_GRAY2BINARY_THRESH"] >> BLUE_GRAY2BINARY_THRESH;
    sentinel_detect["SENTINEL_RED_SUBTRACT_THRESH"] >> RED_SUBTRACT_THRESH;
    sentinel_detect["SENTINEL_BLUE_SUBTRACT_THRESH"] >> BLUE_SUBTRACT_THRESH;

    sentinel_detect["SENTINEL_ROI_ENABLE"] >> ROI_ENABLE;
}