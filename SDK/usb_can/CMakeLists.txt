cmake_minimum_required(VERSION 3.10)
project(hw)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加include目录
include_directories(include include/lcm_related)
include_directories(/usr/share/c++-mscl/source)
include_directories(/usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)

# 添加Eigen库的头文件路径
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 设置安装路径
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "/usr/share/c++-mscl")

# 目标可执行文件的源文件
set(SOURCES
    main.cpp
    src/MotorControl.cpp 
    src/LegControl.cpp 
    src/RobotRunner.cpp
    src/LordImu.cpp
    src/HelpFunction.cpp
)

# 添加可执行文件
add_executable(hw ${SOURCES})

# 链接共享库
target_link_libraries(hw 
    -lpthread
    usb_can
    utilities
    lcm
    mscl
)

# 添加调试信息
set(CMAKE_BUILD_TYPE Debug)