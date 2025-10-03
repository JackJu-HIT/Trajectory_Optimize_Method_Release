/*
 * @Function:Trajectory Optimize Method Manager
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */
#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "vertexTImeDiff.h"
#include "edge_obstacle_edge.h"
#include "edge_via_point.h"
#include "tools.h"
#include "velocityEdge.h"
#include "kinetic_edge.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread/once.hpp>
#include <stdexcept>
#include <limits>

// -----------------------------------------------------------------------------
// 优化器管理类
// -----------------------------------------------------------------------------
namespace teb_local_planner
{

class plannerManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 构造函数：初始化配置、优化器、顶点容器
    plannerManager(const TebConfig& cfg)
        : cfg_(cfg), vertexId_(0)
    {
        optimizer_ = initOptimizer();
        if (!optimizer_) 
        {
            throw std::runtime_error("严重错误：optimizer_ 初始化失败！");
        }
        pose_vertices_.clear();
        timediff_vec_.clear();
    }

    // 析构函数：释放顶点内存
    ~plannerManager()
    {
        clearGraph();
    }

    // 设置路径信息（仅保存路径，不创建顶点）
    void setpathInfo(std::vector<tools::pathInfo>& path);

    // 设置障碍物信息
    void setObstacleInfo(std::vector<tools::obstacleInfo>& obs);

    // 获取优化结果
    void getPlannerResults(std::vector<tools::pathInfo>& path);

    // 核心：运行优化流程
    void runOptimization();

private:
    // 初始化优化器（注册类型、配置求解器）
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // 注册g2o自定义类型（顶点+边）
    static void registerG2OTypes();

    // 添加轨迹顶点（ID从0开始）
    void AddVertices();

    // 添加时间差顶点（ID延续轨迹顶点）
    void AddTimeDiffVertices();

    // 添加障碍物约束边
    void AddObstacleEdges();

    // 添加途经点约束边
    void AddViaPointEdges();

    // 添加速度约束边
    void AddVelocityEdgs();

    // 添加运动学约束边
    void AddEdgeKinematics();

    // 查找最近的轨迹顶点（辅助途经点约束）
    int findClosestTrajectoryPose(tools::pathInfo &ref_point, int& idx);

    // 执行单次优化迭代
    bool optimizeGraph();

    // 清空图（释放内存+重置状态）
    void clearGraph();

    // 打印优化结果
    void printResult();

private:
    const TebConfig& cfg_;                                  // 全局配置（只读）
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_;     // g2o优化器
    std::vector<VertexPoint2D*> pose_vertices_;             // 轨迹顶点容器（x,y,theta）
    std::vector<vertexTimeDiff*> timediff_vec_;             // 时间差顶点容器
    std::vector<tools::pathInfo> pathPointArr_;             // 原始路径点
    std::vector<tools::obstacleInfo> obstaclePointInfo_;    // 障碍物信息
    int vertexId_;                                          // 顶点ID计数器（确保唯一）
};

}  // namespace teb_local_planner