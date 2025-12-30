# Trajectory_Optimize_Method_Release

这是一个基于 **Timed Elastic Band (TEB) 算法** 原理实现的轨迹优化器 C++ 项目。本项目旨在帮助开发者深入理解 TEB 算法的核心思想，并提供一个简洁、可扩展的代码框架，方便进行二次开发和集成。

---

## 🚀 效果演示 (Demo)

下图展示了优化器在给定路径点、起点、终点和障碍物的情况下，生成的平滑且无碰撞的最优轨迹。

<p align="center">
  <img src="https://github.com/JackJu-HIT/Trajectory_Optimize_Method_Release/blob/master/result.png?raw=true" width="600"/>
</p> 

*注：图中红色虚线为初始路径，蓝色实线为 TEB 优化后的轨迹。*

---

## 🛠️ 构建与运行 (How to build & run)

### 依赖 (Dependencies)
*   **Eigen3**: 用于矩阵运算。请确保已安装。
    ```bash
    sudo apt install libeigen3-dev
    ```

### 构建步骤
本项目使用 CMake 构建，请在终端中执行以下命令：

```bash
# 1. 创建构建目录
mkdir build

# 2. 进入目录并配置
cd build
cmake ..

# 3. 编译
make

# 4. 运行示例
./main
