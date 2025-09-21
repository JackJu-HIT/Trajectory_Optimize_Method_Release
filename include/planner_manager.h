
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


// -----------------------------------------------------------------------------
// 优化器管理类
// -----------------------------------------------------------------------------
namespace teb_local_planner
{


class plannerManager
{
public:
    // 构造函数：初始化配置、优化器、顶点容器
    plannerManager(const TebConfig& cfg)
        : cfg_(cfg)
    {
        optimizer_ = initOptimizer();
        pose_vertices_.clear();
        timediff_vec_.clear();
        vertexId_ = 0;
    }

    // 析构函数：释放顶点内存
    ~plannerManager()
    {
       clearGraph();
    }

    void setpathInfo(std::vector<tools::pathInfo>& path);

    void setObstacleInfo(std::vector<tools::obstacleInfo>& obs);

    void getPlannerResults(std::vector<tools::pathInfo>& path);

    // 核心：运行优化
    void runOptimization();
private:
    // 初始化优化器（注册类型、配置求解器）
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // 注册g2o类型（顶点+边）
    static void registerG2OTypes();

    // 添加顶点（TEB风格：容器管理顶点）
    void AddVertices();


    // 添加边（TEB风格：new边+setTebConfig+关联顶点）
    void AddObstacleEdges();

    void AddViaPointEdges();

    void AddVelocityEdgs();

    void AddEdgeKinematics();

    int findClosestTrajectoryPose(tools::pathInfo &ref_point,int& idx);

    // 执行优化
    bool optimizeGraph();

    // 清空图（保留顶点，仅删除边）
    void clearGraph()
    {
        if (optimizer_)
        {
            optimizer_->edges().clear();
            optimizer_->clear();
        }
    }

    // 打印优化结果
    void printResult();

private:
    const TebConfig& cfg_;  // 全局配置（只读）
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_;  // 优化器
    std::vector<VertexPoint2D*> pose_vertices_;          // 顶点容器
    std::vector<vertexTimeDiff*> timediff_vec_;
    std::vector<tools::pathInfo> pathPointArr_;
    std::vector<tools::obstacleInfo> obstaclePointInfo_;
    int vertexId_;
};
}  // namespace teb_local_planner
