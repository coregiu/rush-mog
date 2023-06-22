cmake_minimum_required(VERSION 3.10)

# 项目名称
set(PROJECT_NAME "RUCAR")

include (CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)
CMAKE_FORCE_C_COMPILER(sdcc SDCC)
# set(CMAKE_C_COMPILER sdcc)

project(${PROJECT_NAME} C)  # 这个C是必须的, 否则将会提示CXX编译器错误

# 头文件查找目录
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/pt)

# 源码子目录
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
