
/*
 * @Function: edgeKineticConstraint Contraint Edge Class
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-21 12:47:01
 */

#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"
#include "tools.h"
#include "vertexTImeDiff.h" 

///距离约束边（继承TEB二元边基类）
namespace teb_local_planner
{

class EdgeKineticConstraint : public BaseTebMultiEdge<2,Eigen::Vector2d>// 2维残差，类型Eigen::Vector2d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* @brief 构造函数：指定连接的顶点数量（3个）
     */
    EdgeKineticConstraint()
    {
        resize(2);  // 关键：连接3个顶点（v1, v2, v3）
    }

    // 核心：误差计算（使用基类的cfg_获取配置）
    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeViaPointConstraint!" << std::endl;
            _error[0] = 0.0;
            return;
        }
        // 获取顶点
        const VertexPoint2D* v1 = static_cast<const VertexPoint2D*>(_vertices[0]);
        const VertexPoint2D* v2 = static_cast<const VertexPoint2D*>(_vertices[1]);
      
        Eigen::Vector3d point1 = v1->estimate();
        Eigen::Vector3d point2 = v2->estimate();
        
        Eigen::Vector2d p1(point1[0],point1[1]);
        Eigen::Vector2d p2(point2[0],point2[1]);
        
        Eigen::Vector2d  deltaS = p2 - p1;
        // non holonomic constraint
        _error[0] = fabs( ( cos(point1[2])+cos(point2[2]) ) * deltaS[1] - ( sin(point1[2])+sin(point2[2]) ) * deltaS[0] );

        // positive-drive-direction constraint
        Eigen::Vector2d angle_vec ( cos(point1[2]), sin(point1[2]) );	   
        _error[1] = tools::penaltyBoundFromBelow(deltaS.dot(angle_vec), 0,0);
        std::cout << "EdgeKineticConstraint _error" << _error[0] << std::endl;
    }

    void setcfg(const TebConfig* cfg)
    {
        cfg_         = cfg;
    }
};
}  // namespace teb_local_planner
