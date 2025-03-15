cmake_minimum_required(VERSION 3.10)
project(SDUOG-sim2sim)

# 设置编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 查找LCM库
find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)  # 新增Eigen3查找

# Webots路径配置
set(WEBOTS_HOME /usr/local/webots)
set(EIGEN3_INCLUDE_DIRS "/usr/local/include/eigen3")
include_directories(${WEBOTS_HOME}/include/controller/c)
link_directories(${WEBOTS_HOME}/lib/controller)

# 项目包含目录
include_directories(
    include
    ${LCM_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
)

include_directories(./deploy/deploy_lcm/msg_lcm)

# 源文件列表
file(GLOB_RECURSE SOURCE_FILES 
    "src/*.cpp"
)

# 创建可执行文件
add_executable(SDUOG-sim2sim-webots 
    ${SOURCE_FILES}
)

# 链接库配置
target_link_libraries(SDUOG-sim2sim-webots
    Controller
    lcm
    pthread  # 如果使用多线程需要添加
)

# 现代CMake风格包含目录配置
target_include_directories(SDUOG-sim2sim-webots PRIVATE
    include
    ${LCM_INCLUDE_DIRS}
    ${WEBOTS_HOME}/include/controller/c
    ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
)

# 安装规则（可选）
install(TARGETS SDUOG-sim2sim-webots
    RUNTIME DESTINATION bin
)