/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include "base_teb_edges.h"
// -----------------------------------------------------------------------------
// 3. 自定义顶点
// -----------------------------------------------------------------------------
namespace teb_local_planner
{
/**
 * 在 g2o 框架中，g2o::BaseVertex<2, Eigen::Vector2d> 中的 2 表示该顶点（Vertex）所代表的优化变量的维度。
具体来说：
第一个模板参数 2：指定了待优化变量的自由度（维度）。对于二维点顶点（Eigen::Vector2d），其包含 x 和 y 两个分量，因此优化变量的维度是 2。
第二个模板参数 Eigen::Vector2d：指定了存储优化变量的数据类型（即顶点的估计值类型）。
举例说明：
若定义一个三维点顶点，会写成 g2o::BaseVertex<3, Eigen::Vector3d>（维度为 3，对应 x, y, z）。
若定义一个旋转角度顶点（单变量），会写成 g2o::BaseVertex<1, double>（维度为 1，对应角度值）。
这个 2 本质上是告诉 g2o 框架：该顶点有 2 个需要被优化求解的变量，框架会根据这个维度来分配内存、构建雅可比矩阵和 Hessian 矩阵，从而完成优化计算。***/
class vertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置顶点为原点
    virtual void setToOriginImpl() override { _estimate = 0;}//_estimate.setZero(); }

    // 顶点更新：应用增量
    virtual void oplusImpl(const double* update) override
    {
        _estimate += update[0];// Eigen::Vector2d(update[0], update[1]);
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return true; }
};
}  // namespace teb_local_planner
