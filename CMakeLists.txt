cmake_minimum_required(VERSION 2.8)
project(TrajectoryOptimizeMethod)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

#link_directories(/usr/include/python2.7)
include_directories(/usr/include/python3.12)
set(NUMPY_INCLUDE_DIR "/usr/lib/python3/dist-packages/numpy/core/include/")
include_directories(
    ${NUMPY_INCLUDE_DIR})
# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

include_directories(
    ${Boost_INCLUDE_DIRS}  # 添加 Boost 头文件目录
)

include_directories(include)

add_executable(main main.cpp src/planner_manager.cpp)
target_link_libraries(main  ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}   ${Python3_LIBRARIES}    ${Boost_LIBRARIES}  # 链接 Boost 库
)

