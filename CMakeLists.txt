cmake_minimum_required(VERSION 4.1)
project(TrajectoryOptimizeMethod)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost 1.89 REQUIRED)

# 手动指定系统 Python 路径（请根据你的实际路径修改）
set(PYTHON3_EXECUTABLE "/opt/homebrew/bin/python3")  # Python 可执行文件路径
set(PYTHON3_INCLUDE_DIR "/opt/homebrew/opt/python@3.13/Frameworks/Python.framework/Versions/3.13/include/python3.13")  # Python 头文件目录
set(PYTHON3_NUMPY_INCLUDE_DIR "/opt/homebrew/lib/python3.13/site-packages/numpy/_core/include")  # Numpy 头文件目录
include_directories(${PYTHON3_INCLUDE_DIR} ${PYTHON3_NUMPY_INCLUDE_DIR})
# 查找 Python3 开发环境
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# 手动指定 Eigen 安装路径（替换为你的实际路径）
set(EIGEN3_INCLUDE_DIR "/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/Eigen")  # 这里是包含 Eigen 文件夹的上级目录
find_package(Eigen3 REQUIRED)

# 确保编译器能找到 Eigen 头文件
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
    ${Boost_INCLUDE_DIRS}  # 添加 Boost 头文件目录
)

include_directories(include)

add_executable(main main.cpp src/planner_manager.cpp)
target_link_libraries(main  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}   ${Python3_LIBRARIES}    ${Boost_LIBRARIES}  # 链接 Boost 库
)

