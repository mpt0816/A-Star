# CMake 最低版本号要求
cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11 -g)

# 项目信息
project(AStar)

# 生成目标
add_executable(AStar src/main.cpp src/search.cpp)