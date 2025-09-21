
/*
 * @Function: Obstacle Contraint Edge Class
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */

#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

// 距离约束边（继承TEB二元边基类）
namespace teb_local_planner
{
//public BaseTebUnaryEdge<1, teb_local_planner::obstacleInfo, VertexPoint2D>
//第一个参数 1：表示误差向量的维度（这里是 1 维误差）。
//第二个参数 double：表示边所关联的测量值（observation）的数据类型。
//第三个参数 VertexPoint2D：表示该边所连接的顶点类型（这里连接的是一个 2D 点顶点）。
class EdgeObstacleConstraint : public BaseTebUnaryEdge<1, tools::obstacleInfo, VertexPoint2D>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 核心：误差计算
    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeObstacleConstraint!" << std::endl;
            _error[0] = 0.0;
            return;
        }

        // 获取顶点
        const VertexPoint2D* v1 = static_cast<const VertexPoint2D*>(_vertices[0]);
        Eigen::Vector3d point = v1->estimate();
        
        double dist = sqrt(pow(_measurement.x - point[0],2) + pow(_measurement.y - point[1],2));
        _error[0] = tools::penaltyBoundFromBelow(dist, cfg_->min_obstacle_dist, cfg_->penalty_epsilon);

        // std::cout <<  "point" << point <<  "误差值: " << _error[0] << std::endl;  // 距离小于阈值时应>0
    }

    void setObstcele(tools::obstacleInfo &obs,const TebConfig* cfg)
    {
        _measurement = obs;
        cfg_         = cfg;
    }
};
}  // namespace teb_local_planner
