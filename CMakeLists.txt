cmake_minimum_required(VERSION 3.5.1)
project(HERORM2021)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 设置第三方库文件夹位置
set(OPENCV_DIR /usr/local/share/OpenCV)

# 搜索库
find_package(OpenCV REQUIRED)

# 添加源文件
file(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable(${PROJECT_NAME} ${sourcefiles})

# 添加头文件
# 递归添加头文件函数实现
function(include_sub_dir root_dir)
    if (IS_DIRECTORY ${root_dir}) 
        # message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()
    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_dir(${root_dir}/${sub})
        endif()
    endforeach()
endfunction()
#调用递归函数
include_sub_dir(${CMAKE_SOURCE_DIR}/src) 
include_sub_dir(${CMAKE_SOURCE_DIR}/Config) 

include_directories(${OpenCV_INCLUDE_DIRS})

# 添加需要链接的库文件目录
link_directories(./src/auto_aim/armor_detect/classifier/darknet/)

# 添加链接库
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        libdarknet.so
        -pthread
        -lMVSDK
        -fopenmp
        /lib/libMVSDK.so)


