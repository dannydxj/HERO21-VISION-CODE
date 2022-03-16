# HERORM2021

HERO-RoboMaster 团队 2021 赛季统一代码框架。所有机器人的视觉代码全部出自该框架。

项目地址：https://github.com/skywalker-dell/HERO21-VISION-CODE

负责人：

- 朱纹轩  [![img](https://img.shields.io/badge/github-skywalker--dell-green.svg?logo=github)](https://github.com/skywalker-dell)
- 董行健  [![img](https://img.shields.io/badge/github-dannydxj-green.svg?logo=github)](https://github.com/dannydxj)

## 环境依赖

- Ubuntu 16.04 及以上
- OpenCV 3.4 & OpenCV Contrib 3.4
- CMake 3.5.1 及以上

## 编译运行

```sh
git clone ${this project}
cd HERO21-VISION-CODE
mkdir build
cd build
cmake ..
make
./HERORM2021
```

## 项目结构说明

```
.
├── Config                             
└── src
    ├── aimbot							
    │   ├── antigyro					
    │   ├── armor						
    │   ├── armor_detect				
    │   │   ├── findlightbars
    │   │   ├── grade_armors
    │   │   ├── image_process
    │   │   └── match_armors
    │   ├── armor_predict				
    │   ├── classifier					
    │   │   ├── classifier_dk
    │   │   │   └── darknet
    │   │   └── classifier_sj
    │   │       └── para
    │   ├── others
    │   │   ├── init
    │   │   └── roi
    │   └── pick_target					
    │       ├── search_armor
    │       └── track_armor
    ├── camera							
    │   └── mvcamera
    ├── communication					
    │   ├── can
    │   └── can_to_debug
    ├── hit_sentinel					
    ├── runesolver						
    │   ├── fit_rune_motion
    │   ├── runedetector
    │   └── runepredictor
    ├── target_solve					
    ├── tools							
    │   ├── debugger
    │   ├── macros
    │   ├── queues
    │   ├── timer
    │   └── util_func
    └── workspace						
        ├── commu_thread				
        │   ├── receive_thread
        │   └── send_thread
        ├── get_img_thread				
        ├── img_process_thread			
        │   ├── armor_func
        │   ├── rune_func
        │   └── sentinel_func
        └── others
```

## 模块介绍

### Config

配置文件说明

### aimbot

自瞄系统，包括装甲板类、装甲板检测、数字识别、运动预测、数字识别等多个模块

### camera

对迈德威视相机 API 的再封装

### communication

can 通信封装

### hit_sentinel

该模块用于定点击打轨道上运动的哨兵。

### runesolver

能量机关模块，包括椭圆拟合、能量机关装甲板识别和能量机关运动预测等多个模块。

### target_solve

坐标解算模块，用于计算装甲板的三维坐标。

### workspace

主程序流程控制模块，其中包含对三个线程的调度：

- 通信线程
- 图像接收线程
- 图像处理线程

### tools

常用的工具代码。
