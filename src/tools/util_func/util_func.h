/**
 * @file util_func.h
 * @brief 工具类
 * @details 存放一些通用函数和常量
 */

#ifndef UTIL_H
#define UTIL_H

#include <opencv2/opencv.hpp>
#include "types.h"
#include "log.h"

/**
 * @brief 工具类
　* 存放常计算量，坐标转换工具，欧式距离计算工具等
 */
class Util {
public:
   /// 数学常量, π
   constexpr static double PI = 3.1415926;

   /// 物理常量, 重力加速度
   constexpr static double G = 9.7988;

   /// 自然对数
   constexpr static double e = 2.718281828;

   // 浮点数精度控制
   constexpr static double EXP = 0.01;

   /// x轴补偿值，单位为毫米
   constexpr static double X_OFFSET = 0.0;

   /// y轴补偿值，单位为毫米
   constexpr static double Y_OFFSET = -107.518;

   /// z轴补偿值，单位为毫米
   constexpr static double Z_OFFSET =  73.771;

   /// 自瞄pitch补偿量，分辨率 1/10 度
   static double pitch_skew_on_armor;

   /// 自瞄yaw补偿量， 分辨率 1/10度
   static double yaw_skew_on_armor;

   /// 能量机关pitch补偿量，分辨率 1/10 度
   static double pitch_skew_on_rune;

   /// 能量机关yaw补偿量， 分辨率 1/10度
   static double yaw_skew_on_rune;

   /// 小弹丸空气阻力系数（f = -k*v^2）
   constexpr static double k = 45.857 * 0.00001; //TODO

   /// 小弹丸质量
   constexpr static double m = 41 * 0.001;

public:
    /**
     * @brief 获取二维点的欧氏距离
     * @param point1 点1
     * @param point2 点2
     * @return 两点欧氏距离
     */
    static double distance(const cv::Point2d &point1, const cv::Point2d &point2);

    /**
     * @brief 获取三维点的欧式距离
     * @param point1 点1
     * @param point2 点2
     * @return 两点欧氏距离
     */
    static double distance(const cv::Point3d &point1, const cv::Point3d &point2);

    /**
     * @brief 计算子弹的飞行时间,单位是s
     */
    static double computFlightTime(const Target &target, double v);

    /**
     * @brief 根据当前的目标，获得云台yaw轴设定值, 单位度
     */
    static double getPredYaw(const Target &target); // TODO

    /**
     * @brief 根据当前的目标，获得云台pitch轴设定值, 单位度
     */
    static double getPredPitch(const Target &target, double v);

   /**
    * @brief 将单通道图片转换成三通道
    */
    static cv::Mat convertTo3Channels(const cv::Mat &binImg);

    /**
     * 判断浮点数是否约等于0
     * @param x 浮点数
     * @return 该浮点数是否约等于0
     */
    static bool equalZero(double x);

    /**
     * @brief 计算输入两个点构成向量的角度，逆时针为正，0-360
     */
    static double getpolarAngle(const cv::Point2d &origin_point, const cv::Point2d &temp_point);

    /**
     * @brief 平面坐标系只做旋转变换
     */
    static void transAngle(double &x1, double &x2, double theta);

    /**
     * @brief 从摩擦轮坐标系转换到大地坐标系
     * @details 只做旋转变换
     */
    static void wheel2land(Target &target, const ImageClass &image_object);

    /**
     * @brief 返回矩形中的点
     */
    static void extractPoints(cv::RotatedRect &rect, std::vector<cv::Point> &points);

    /**
     * @brief 在图像上绘画旋转矩形
     * @param image 源图像
     * @param rect 旋转矩形
     * @param scalar 颜色, 默认为红色
     * @param thickness 线条粗细, 默认为2
     */
    static void drawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &scalar = cv::Scalar(0, 0, 255), int thickness = 2);

   /**
    * @brief 将一个字节中的指定位置 1 （flag = 1）或者置 0 （flag = 0）
    * @param byte 要修改的字节（unsigned char类型）
    * @param start_index 字节中开始修改的下标（注意最右边的下标是 0，向左递增）
    * @param len 修改的长度（注意是往前算的）
    * @param flag 1 将选择位置 1，0 将选择位置 0
    */
   static void setByteFlags(unsigned char &byte, int start_index, int len, bool flag);

   /**
    * @brief 获取字节中指定的几位
    */
   static unsigned char readByteFlags(unsigned char byte, int start_index, int len);

   /**
    * @brief 获得一个点的绝对三维空间位置
    */
   static double getAbsolute3Ddistance(const Target &target);
};

#endif  // UTIL_H
