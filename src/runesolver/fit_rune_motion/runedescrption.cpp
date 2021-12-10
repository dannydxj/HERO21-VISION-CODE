#include "runedescription.h"

void RuneDescriptior::onEnter() {}

void RuneDescriptior::runSmallRune(ImageClass &image_object, const ReadPack &mcu_data, const std::vector<Candidate_Rect> &candidate_rects, int energy_yaw)
{
    if (mcu_data.isRightMouseButtonClicked) /* 有触发且线程工作,需要清空数据点, 重新拟合 */
    { 
        clear();
    }
    if (candidate_rects.empty())
    { /* 没有候选矩形，直接返回不拟合 */
        return;
    }
    double pitch = image_object.pose.ptz_pitch;
    double yaw = image_object.pose.ptz_yaw;
    double roll = image_object.pose.ptz_roll;
    std::vector<Target> targets;
    /* 计算三维坐标 */
    RunesolvePnP4Points(candidate_rects, targets);
    /* 坐标系转换成大地坐标系，右手坐标系，y轴为深度信息 */
    RunetransCoordinate(targets, pitch, yaw, roll, energy_yaw);
    /* 因为聚集点判断会删掉点，所以需要判断targets是否为空 */
    if (!targets.empty())
    {
        for (auto iter = targets.begin(); iter != targets.end(); ++iter)
        {
            iter->clock = image_object.pose.mcu_time;
            points.push(*iter); /* 更新历史点，所有扇叶 */
        }
        if (!flag_handle_density && judgeFansSwitched(targets.at(0), last_target)) // 一开始一定会清空（last_target数据都为0）
        {
            single_fan_points.clear(); /* 保证仅存储当前扇叶点 */
        }
        single_fan_points.push(ptz_target);
    }

    if(calibrate(points, c, radius)) // 只有拟合圆成功之后，才会判断角速度方向， 计算角速度
    {
        flag_fit_circle_success = 1;
        // 旋转方向判断
        judgeRotateDir();
    }else{
        flag_fit_circle_success = 0;
    }
    setConstRuneData(image_object.pose.mcu_time);
    if (!targets.empty())
    {
        last_target = ptz_target;
    }
}

void RuneDescriptior::runBigRune(ImageClass &image_object, const ReadPack &mcu_data, const std::vector<Candidate_Rect> &candidate_rects, int energy_yaw)
{
    if (mcu_data.isRightMouseButtonClicked) /* 有触发且线程工作,需要清空数据点,重新拟合 */
    { 
        clear();
    }
    if (candidate_rects.empty())
    { /* 没有候选矩形，直接返回不拟合 */
        return;
    }
    double pitch = image_object.pose.ptz_pitch;
    double yaw = image_object.pose.ptz_yaw;
    double roll = image_object.pose.ptz_roll;
    std::vector<Target> targets;
    /* 计算三维坐标 */
    RunesolvePnP4Points(candidate_rects, targets);
    /* 坐标系转换成大地坐标系，右手坐标系，y轴为深度信息 */
    RunetransCoordinate(targets, pitch, yaw, roll, energy_yaw);
    /* 因为聚集点判断会删掉点，所以需要判断targets是否为空 */
    if (!targets.empty())
    {
        for (auto iter = targets.begin(); iter != targets.end(); ++iter)
        {
            iter->y = STANDARD_LENGTH;
            iter->clock = image_object.pose.mcu_time;
            points.push(*iter); /* 更新历史点，所有扇叶 */
        }
        if (!flag_handle_density && judgeFansSwitched(targets.at(0), last_target)) // 一开始一定会清空（last_target数据都为0）
        {
            single_fan_points.clear(); /* 保证仅存储当前扇叶点 */
        }
        single_fan_points.push(ptz_target);
    }

    if(calibrate(points, c, radius)) // 只有拟合圆成功之后，才会判断角速度方向， 计算角速度
    {
        flag_fit_circle_success = 1;
        // 旋转方向判断
        judgeRotateDir();
        // 角速度滤波计算,求解时间零点
        computePalstanceFliter();
    }else{
        flag_fit_circle_success = 0;
    }
    setPivRuneData(image_object.pose.mcu_time);
    if (!targets.empty())
    {
        last_target = ptz_target;
    }
}



void RuneDescriptior::RunesolvePnP4Points(const std::vector<Candidate_Rect> &todo_candidate_rects, std::vector<Target> &targets) 
{
    for (auto iter = todo_candidate_rects.begin(); iter != todo_candidate_rects.end(); ++iter) 
    {
        cv::RotatedRect selected_rect = iter->rect;
        cv::Point2f vertices[4];
        selected_rect.points(vertices);
        float dis;
        cv::Point2f temp;
        std::vector<cv::Point2f> image_Points;
        // 按顺时针重新排列四个角点
        for (int i = 0; i < 4; ++i) image_Points.emplace_back(vertices[i]);
        dis = static_cast<float>(sqrt(pow(image_Points[0].x - image_Points[1].x, 2) + pow(image_Points[0].y - image_Points[1].y, 2)));
        // 保证（0 - 1）之间为宽度，这是最关键的
        if (abs(dis - selected_rect.size.height) > 0.5) {
            temp = image_Points[0];
            image_Points.erase(image_Points.begin());
            image_Points.emplace_back(temp);
        }
        // 感觉好像没有必要，加上保险
        if (image_Points[0].x > image_Points[1].x) 
        {
            temp = image_Points[0];
            image_Points.erase(image_Points.begin());
            image_Points.emplace_back(temp);
            temp = image_Points[0];
            image_Points.erase(image_Points.begin());
            image_Points.emplace_back(temp);
        }

        std::vector<cv::Point3f> points3d;
        points3d.clear();
        points3d.emplace_back(cv::Point3d(-half_w, -half_h, 0));
        points3d.emplace_back(cv::Point3d(half_w, -half_h, 0));
        points3d.emplace_back(cv::Point3d(half_w, half_h, 0));
        points3d.emplace_back(cv::Point3d(-half_w, half_h, 0));

        Target target;
        solvePnP(points3d, image_Points, CAMERA_MATRIX, DISTORTION_COEFF, rotate_mat, trans_mat, false, CV_EPNP);
        target.x = (trans_mat.at<double>(0, 0) + Util::X_OFFSET) / 1000;  //单位m
        target.y = (trans_mat.at<double>(1, 0) + Util::Y_OFFSET) / 1000;
        target.z = (trans_mat.at<double>(2, 0) + Util::Z_OFFSET) / 1000;
        targets.emplace_back(target);
    }
}

void RuneDescriptior::RunetransCoordinate(std::vector<Target> &targets, double pitch, double yaw, double roll, double energy_yaw) 
{
    for (auto iter = targets.begin(); iter != targets.end(); ++iter) 
    {
        // 转换成电控的坐标系惯用方向
        double temp = iter->y;
        iter->y = iter->z;
        iter->z = -temp;

        // 转换到大地系，此时pitch零参考为水平面， yaw零参考为陀螺仪的零点
        Util::transAngle(iter->y, iter->z, -pitch);
        Util::transAngle(iter->x, iter->z, -roll);  // TODO 暂未用到
        Util::transAngle(iter->x, iter->y, -yaw);

        // std::cout << "大地坐标系x：" << iter->x << std::endl;
        // std::cout << "大地坐标系y："<< iter->y << std::endl;
        // std::cout << "大地坐标系z:" << iter->z << std::endl;

        // energy_yaw 为机器人连线到能量机关平面做垂线与机器人陀螺仪的夹角 + 90
        // 度，极轴为机器人陀螺仪零点，逆时针为正
        // 经过这个转换后，现在的坐标系y-o-z平行于能量机关平面，x-o-z
        // 垂直于能量机关平面
        Util::transAngle(iter->x, iter->y, energy_yaw);
        double K = ((STANDARD_LENGTH) / iter->x);  // 注意是 iter->x

        //根据视角转换
        double tempy = iter->y;
        double tempz = iter->z;

        // 但是最终要拟合的圆时候用的还是x-o-z平行于能量机关平面，y-o-z
        // 垂直于能量机关平面 拟合的圆平行于能量机关平面
        iter->x = -(tempy * K);
        iter->z = tempz * K;
        iter->y = STANDARD_LENGTH;
        // std::cout << "yaw偏转后x：" << iter->x << std::endl;
        // std::cout << "yaw偏转后y：" << iter->y << std::endl;
        // std::cout << "yaw偏转后z：" << iter->z << std::endl;
    }

    // 设置待激活装甲板的坐标,注意要在密集点处理之前做，因为密集点处理会删掉待激活装甲板的点
    ptz_target.x = targets.at(0).x;
    ptz_target.y = targets.at(0).y;  // y不用发送，为标准值
    ptz_target.z = targets.at(0).z;
    rune_info_file << ptz_target.x << " " << ptz_target.y << " " << ptz_target.z << std::endl;

    // 对于密集点进行处理，只需要处理未激活的即可
    flag_handle_density = false;
    double delta_dis = Util::distance(cv::Point2d(ptz_target.x, ptz_target.z), cv::Point2d(last_target.x, last_target.z));
    if (delta_dis < min_delta_dis) 
    {
        // 删掉第一个点，也就是待激活点
        // std::cout << "进入慢速模式：" << std::endl;
        targets.erase(targets.begin());
        flag_handle_density = true;
    } 
}

bool RuneDescriptior::calibrate(CircleQueue<Target, 301> &m_Points, cv::Point2d &Centroid, double &dRadius) 
{
    if (m_Points.length != 0) 
    {
        if (m_Points.length < 40)
        {
            return false;
        }

        double X1 = 0.0;
        double Y1 = 0.0;
        double X2 = 0.0;
        double Y2 = 0.0;
        double X3 = 0.0;
        double Y3 = 0.0;
        double X1Y1 = 0.0;
        double X1Y2 = 0.0;
        double X2Y1 = 0.0;
        for (int i = m_Points.head; i != m_Points.tail; i = (i + 1) % m_Points.size) 
        {
            X1 = X1 + m_Points.values[i].x;
            Y1 = Y1 + m_Points.values[i].z;
            X2 = X2 + m_Points.values[i].x * m_Points.values[i].x;
            Y2 = Y2 + m_Points.values[i].z * m_Points.values[i].z;
            X3 = X3 + m_Points.values[i].x * m_Points.values[i].x * m_Points.values[i].x;
            Y3 = Y3 + m_Points.values[i].z * m_Points.values[i].z * m_Points.values[i].z;
            X1Y1 = X1Y1 + m_Points.values[i].x * m_Points.values[i].z;
            X1Y2 = X1Y2 + m_Points.values[i].x * m_Points.values[i].z * m_Points.values[i].z;
            X2Y1 = X2Y1 + m_Points.values[i].x * m_Points.values[i].x * m_Points.values[i].z;
        }
        double C = 0.0;
        double D = 0.0;
        double E = 0.0;
        double G = 0.0;
        double H = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;

        C = m_Points.length * X2 - X1 * X1;
        D = m_Points.length * X1Y1 - X1 * Y1;
        E = m_Points.length * X3 + m_Points.length * X1Y2 - (X2 + Y2) * X1;
        G = m_Points.length * Y2 - Y1 * Y1;
        H = m_Points.length * X2Y1 + m_Points.length * Y3 - (X2 + Y2) * Y1;
        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);

        if (std::isnan(a) || std::isnan(b)) 
            return false;

        c = -(a * X1 + b * Y1 + X2 + Y2) / m_Points.length;
        double A = 0.0;
        double B = 0.0;
        double R = 0.0;
        A = a / (-2);
        B = b / (-2);
        R = double(sqrt(a * a + b * b - 4 * c) / 2);
        if (std::isnan(R)) 
            return false;

        Centroid.x = A;
        Centroid.y = B;
        dRadius = R;
        // std::cout << "拟合的半径:" << R << std::endl;   
        return true;
    } else
    {
        return false;
    }
}

bool RuneDescriptior::judgeFansSwitched(const Target &curr_target, const Target &last_target)
{
    cv::Point2d curr_point = cv::Point2d(curr_target.x, curr_target.y);
    cv::Point2d last_point = cv::Point2d(last_target.x, last_target.y);
    double delta_dis = Util::distance(curr_point, last_point);
    if (delta_dis > 0.3)
    {
        return true;
    }else
    {
        return false;
    }
}   

void RuneDescriptior::judgeRotateDir()
{
    if (rotate_dir != 0){  // 之前已经判断过，不需要再次判断
        return;
    }
    if (single_fan_points.length < 21 || flag_fit_circle_success !=1 ) // 单扇叶点数太少或者是没有成功拟合圆
    {
        return;
    }else
    {
        Target tmp_front_target, tmp_back_target;
        single_fan_points.back(tmp_front_target, 1);
        single_fan_points.back(tmp_back_target, 18);
        double tmp_rotate_palastance = computePalstance(tmp_front_target, tmp_back_target);
        if (tmp_rotate_palastance > 0)
        {
            rotate_dir = 1;
        }else
        {
            rotate_dir = -1;
        }
    }
}


// 此函数用于计算时间零点
// 在执行此函数前，会判断是否切换扇叶,从而保证用到的点都是同一个扇叶的点
// 圆拟合成功滞后才会执行此函数
void RuneDescriptior::computePalstanceFliter()
{
    if (flag_speed_clock !=0) // 已经计算除时间零点
        return;
    if (flag_fit_circle_success == 0){
        return;
    }

    Target back_target;
    if(getPointToComputepalstance(back_target)) // 能够取到时间跨度合适的点
    {    
        double last_palastance_filter = S_palstance_filter; // 记录上一帧的角速度,注意，在滤波器赋值之前进行计算
        Target tmp_curr_target;
        single_fan_points.back(tmp_curr_target, 1);
        double tmp_palstance = computePalstance(tmp_curr_target, back_target); 
        if (S_palstance_filter == 0) //初始化 
        {
            S_palstance_filter = tmp_palstance;
            L_palstance_filter = S_palstance_filter;
        }else{
            S_palstance_filter = S_palstance_filter * 0.95 + tmp_palstance * 0.05;
            L_palstance_filter = L_palstance_filter * 0.99 + S_palstance_filter * 0.01;
        }

        if (filter_cnt < 200) // 滤波器在稳定之前，先不进行时间零点的判断
        {
            //do nothing
        } else// 滤波器稳定，先判断方向，且未判断过时间零点，开始判断速度过零点
        {
            if (rotate_dir = 0) /* 转动方向未知 */
            {
                return;
            }
            // 记录上一次的角速度结构体和当前角速度结构体,注意平移速度，角速度相乘异号则为临界点
            if (((last_palastance_filter - 1.305 * rotate_dir) * (S_palstance_filter - 1.305 * rotate_dir)) < 0)
            {
                Target tmp_curr_target;
                single_fan_points.back(tmp_curr_target, 1);
                if (last_palastance_filter > S_palstance_filter) // 减速零点，时间往左平移至加速零点，100ms是滤波滞后补偿
                {
                    speed_clock = static_cast<int64_t>(((tmp_curr_target.clock + back_target.clock)/2)+100) - 1668;
                }else{ // 加速零点
                    speed_clock = static_cast<int64_t>(((tmp_curr_target.clock + back_target.clock)/2)+100);
                }
                flag_speed_clock = 1;
            }else{ // 未找到速度临界点
                // do nothing
            }
        }
        ++filter_cnt;
    }
}


bool RuneDescriptior::getPointToComputepalstance(Target &target)
{
    int end = (single_fan_points.tail + single_fan_points.size - 1) % single_fan_points.size;
    for(int i = end; i != single_fan_points.head; i = ((i+single_fan_points.size-1)%single_fan_points.size))
    {
        Target tmp_curr_target;
        single_fan_points.back(tmp_curr_target, 1);
        int64_t delta_clock = abs(single_fan_points.values[i].clock - tmp_curr_target.clock);
        if (delta_clock < 350 && delta_clock > 250) /* 计算角速度需要较长的时间跨度才不至于抖 */
        {
            target = single_fan_points.values[i];
            return true;
        }
    }
    return false;
}


/* 计算角度之差,单位度 */
double RuneDescriptior::getDeltaAngle(double front_angle, double back_angle)
{
    double delta_angle = 0;
    double tmp_delta_angle = front_angle - back_angle; 
    if (tmp_delta_angle > 360.0)
    {
        delta_angle = tmp_delta_angle - 360.0;
    }else if (tmp_delta_angle < -360.0){
        delta_angle = tmp_delta_angle + 360.0;
    }else{
        delta_angle = tmp_delta_angle;
    }
    return delta_angle;
}


/* 计算角速度，逆时针为正，单位度/s */
double RuneDescriptior::computePalstance(const Target target1, const Target target2)
{
    double angle1, angle2, delta_angle, delta_clock, palstance;
    angle1 = Util::getpolarAngle(cv::Point2d(target1.x, target1.z), c);
    angle2 = Util::getpolarAngle(cv::Point2d(target2.x, target2.z), c);
    delta_angle = getDeltaAngle(angle1, angle2); /* 有跨圈处理 */ 
    delta_clock = target1.clock - target2.clock;
    palstance = (delta_angle / delta_clock);
    return palstance; // 返回角速度
}


double RuneDescriptior::get3Ddistance(const Target &target1, const Target &target2)
{
    cv::Point3d point1 = cv::Point3d(target1.x, target1.y, target1.z);
    cv::Point3d point2 = cv::Point3d(target2.x, target2.y, target2.z);
    double dis = Util::distance(point1, point2);
    return dis;
}

void RuneDescriptior::setConstRuneData(int clock)
{
    const_rune_data.Dir = rotate_dir;
    const_rune_data.radius = radius;
    const_rune_data.x = c.x;
    const_rune_data.z = c.y;
    const_rune_data.target = ptz_target;
    const_rune_data.clock = clock;
}

void RuneDescriptior::setPivRuneData(int clock)
{
    piv_rune_data.Dir = rotate_dir;
    piv_rune_data.radius = radius;
    piv_rune_data.x = c.x;
    piv_rune_data.z = c.y;
    piv_rune_data.target = ptz_target;
    piv_rune_data.clock = clock;
    piv_rune_data.speed_clock = speed_clock;
}


void RuneDescriptior::clear()
{
    points.clear();              /* 用于拟合圆的点集清空 */
    single_fan_points.clear();   /* 记录一个扇叶点集清空 */
    flag_fit_circle_success = 0; /* 圆拟合成功标志 */
    rotate_dir = 0;              /* 转动方向 */
    flag_speed_clock = 0;        /* 寻找到时间零位标志 */
    speed_clock = 0;             /* 时间零位 */
    filter_cnt = 0;              /* 滤波器计算cnt */
}

void RuneDescriptior::init(const cv::FileStorage &file_storage) 
{
    loadParam(file_storage);
    // 输出信息
    rune_info_file.open("../logs/rune.txt", std::ios::out);
    if (!rune_info_file.is_open()) 
    {
        std::cout << "rune file open error!" << std::endl;
    }
}

void RuneDescriptior::loadParam(const cv::FileStorage &file_storage)
{
    file_storage["camera_matrix"] >> CAMERA_MATRIX;
    file_storage["distortion_coeff"] >> DISTORTION_COEFF;

    cv::FileNode rune_solve = file_storage["rune"];
    rune_solve["RUNEARMOR_HALF_WIDTH"] >> half_w;
    rune_solve["RUNEARMOR_HALF_HEIGHT"] >> half_h;
}