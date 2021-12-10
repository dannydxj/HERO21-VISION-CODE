#ifndef CAMERA_H
#define CAMERA_H

#include <unistd.h>

#include <exception>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

/**
 * @brief 相机异常类
 * 相机在比赛中可能出现连接不稳定等异常，写一异常类帮助相机重新连接；
 * 同时防止因为相机异常造成的程序崩溃
 */
class CameraException : public std::exception {
   private:
    /// 异常信息字符串
    std::string e_what;

   public:
    CameraException() = default;

    /**
     * @brief 自定义构造函数, 需要给出异常信息
     *
     * @param error 异常描述信息
     */
    CameraException(const std::string &error) : e_what(error) {}

    ~CameraException() throw() {}

    /**
     * @brief 异常规格说明：不抛出异常
     *
     * @return 异常信息字符串
     * @note 该函数为std::exception类中的覆盖
     */
    virtual const char *what() const throw() { return e_what.c_str(); }
};

#endif  // CAMERA_H