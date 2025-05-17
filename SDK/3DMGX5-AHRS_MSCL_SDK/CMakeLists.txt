cmake_minimum_required(VERSION 3.10)
project(test)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加MSCL库路径和头文件路径
include_directories(/usr/share/c++-mscl/source)
include_directories(/usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)

# 设置运行时库路径
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/share/c++-mscl")

# 添加可执行文件
add_executable(test main.cpp)

# 链接MSCL库和其他需要的库
target_link_libraries(test mscl stdc++ pthread)
