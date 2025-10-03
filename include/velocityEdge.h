
/*
 * @Function: Follw EdgeVelocityConstraint Contraint Edge Class
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */

#pragma once 
#include "base_teb_edges.h"
#include  "vertexPoint.h"
#include "tools.h"
#include "vertexTImeDiff.h" 

///距离约束边（继承TEB二元边基类）
namespace teb_local_planner
{

class EdgeVelocityConstraint : public BaseTebMultiEdge<2,Eigen::Vector2d>// 2维残差，类型Eigen::Vector2d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* @brief 构造函数：指定连接的顶点数量（3个）
     */
    EdgeVelocityConstraint()
    {
        resize(3);  // 关键：连接3个顶点（v1, v2, v3）
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
        const vertexTimeDiff* v3 = static_cast<const vertexTimeDiff*>(_vertices[2]);

        Eigen::Vector3d point1 = v1->estimate();
        Eigen::Vector3d point2 = v2->estimate();

        double dist =  sqrt(pow(point1[0] - point2[0],2) + pow(point1[1] - point2[1],2));
        double deltaAngle = fabs(point1[2] - point2[2]);
        double vel = dist / v3->estimate();
        double w   = tools::normalize_theta(deltaAngle) / v3->estimate();
        // std::cout << "real w =" << w << "real vel =" << vel << std::endl;

        _error[0] =  tools::penaltyBoundToInterval(vel, -cfg_->max_vel_x_backwards, cfg_->max_vel,cfg_->penalty_epsilon);
        _error[1] =  tools::penaltyBoundToInterval(w, cfg_->max_vel_theta,cfg_->penalty_epsilon);
        // std::cout << " vel _error[0] =" <<  _error[0] << " w  _error[1]" <<  _error[1] << std::endl;
    }

    void setcfg(const TebConfig* cfg)
    {
        cfg_         = cfg;
    }
};
}  // namespace teb_local_planner
