// 宏开关，控制输出
#ifndef MACRO_SWITCH_H
#define MACRO_SWITCH_H

// #define ARMOR_DEBUG //输出自瞄的相关参数
// #define RUNE_DEBUG  //输出能量机关相关参数
#define SENTINEL_DEBUG //输出打击哨兵相关参数
// #define CAN_DEBUG   //输出CAN通信相关参数
// #define CAMERA_DEBUG //输出相机相关参数

#ifdef ARMOR_DEBUG
    #define armor_printf printf
#else
    #define armor_printf(...)
#endif

#ifdef RUNE_DEBUG
    #define rune_printf printf
#else
    #define rune_printf(...)
#endif

#ifdef SENTINEL_DEBUG
    #define sentinel_printf printf
#else
    #define sentinel_printf(...)
#endif

#ifdef CAN_DEBUG
    #define can_printf printf
#else
    #define can_printf(...)
#endif

#ifdef CAMERA_DEBUG
#define camera_printf printf
#else
#define camera_printf(...)
#endif

#endif  // MACRO_SWITCH_H