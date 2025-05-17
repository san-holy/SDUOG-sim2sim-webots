cmake_minimum_required(VERSION 3.10)
project(SDUOG-sim2sim)

# 设置编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找LCM库
find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)  # 新增Eigen3查找

# 设置Eigen3头文件路径
set(EIGEN3_INCLUDE_DIRS "/usr/local/include/eigen3")

# 包含目录
include_directories(/usr/share/c++-mscl/source)
include_directories(/usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
include_directories(/usr/include/eigen3)

set(CMAKE_INSTALL_RPATH "/usr/share/c++-mscl")

# 项目包含目录
include_directories(
    include
    SDK
    ${LCM_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
)

include_directories(./deploy/deploy_lcm/msg_lcm)

# 源文件列表
file(GLOB_RECURSE SOURCE_FILES 
    "src/*.cpp"
    "SDK/*.cpp"
)

# 创建可执行文件
add_executable(SDUOG-sim2sim ${SOURCE_FILES})

# 链接库配置
target_link_libraries(SDUOG-sim2sim
    lcm
    pthread  # 如果使用多线程需要添加
    usb_can
)
target_link_libraries(SDUOG-sim2sim mscl)

# 现代CMake风格包含目录配置
target_include_directories(SDUOG-sim2sim PRIVATE
    include
    ${LCM_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
)

# 安装规则（可选）
install(TARGETS SDUOG-sim2sim
    RUNTIME DESTINATION bin
)
