#pragma once 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>  // 必须包含g2o多元边基类头文件
#include <g2o/core/factory.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <boost/thread/once.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <memory>


namespace teb_local_planner
{
// -----------------------------------------------------------------------------
// 存储全局配置：权重、目标位置等
// -----------------------------------------------------------------------------
struct TebConfig
{
    // 软约束配置：目标位置 + 权重
    double min_obstacle_dist = 2;
    double penalty_epsilon   = 0.05;
    double obstacle_weight   = 10;
    double weight_viapoint   = 1;
    double max_vel           = 1.0;
    double max_vel_theta     = 1.0;
    double max_vel_x_backwards = 0.3;
    double weight_max_vel_x    = 1;
    double weight_max_vel_theta = 1;
    double weight_kinematics_nh  = 1;
    double weight_kinematics_forward_drive = 1;

    // 优化器配置
    bool optimization_verbose = true;
    int no_inner_iterations = 5;
    int no_outer_iterations = 4;
};

// -----------------------------------------------------------------------------
// 1. 一元边基类（连接1个顶点）
// -----------------------------------------------------------------------------
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebUnaryEdge() { this->_vertices[0] = nullptr; }

    // 析构函数：从顶点的edges列表中删除当前边（避免野指针）
    virtual ~BaseTebUnaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
    }

    // 统一误差获取接口：计算误差后返回
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现（满足g2o接口要求，子类可重写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口：设置TebConfig
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针（所有子类边可直接访问）
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐
};

// -----------------------------------------------------------------------------
// 2. 二元边基类（连接2个顶点）
// -----------------------------------------------------------------------------
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebBinaryEdge()
    {
        this->_vertices[0] = nullptr;
        this->_vertices[1] = nullptr;
    }

    // 析构函数：从两个顶点的edges列表中删除当前边
    virtual ~BaseTebBinaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
        if (this->_vertices[1])
            this->_vertices[1]->edges().erase(this);
    }

    // 统一误差获取接口
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------
// 3. 多元边基类（新增！连接任意数量顶点，动态绑定）
// -----------------------------------------------------------------------------
template <int D, typename E>  // 多元边无需在模板参数指定顶点类型（动态绑定）
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:
    using typename g2o::BaseMultiEdge<D, E>::ErrorVector;  // 残差向量类型
    using g2o::BaseMultiEdge<D, E>::computeError;          // 继承误差计算接口

    // 构造函数：初始化顶点列表（默认空，需通过resize动态设置顶点数量）
    BaseTebMultiEdge() {}

    // 析构函数：从所有关联顶点的edges列表中删除当前边（核心！避免野指针）
    virtual ~BaseTebMultiEdge()
    {
        for (size_t i = 0; i < this->_vertices.size(); ++i)
        {
            if (this->_vertices[i])  // 检查顶点非空
                this->_vertices[i]->edges().erase(this);
        }
    }

    // 重写resize：动态设置顶点数量时，确保新顶点指针初始化为nullptr
    virtual void resize(size_t num_vertices) override
    {
        g2o::BaseMultiEdge<D, E>::resize(num_vertices);  // 调用父类resize
        // 初始化新添加的顶点指针为nullptr，避免访问随机内存
        for (size_t i = 0; i < this->_vertices.size(); ++i)
        {
            if (!this->_vertices[i])
                this->_vertices[i] = nullptr;
        }
    }

    // 统一误差获取接口：计算误差后返回（与一元/二元边接口一致）
    ErrorVector& getError()
    {
        computeError();  // 先调用子类实现的误差计算
        return this->_error;
    }

    // 序列化默认实现（满足g2o接口要求，子类可按需重写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口：与一元/二元边保持一致，传递TebConfig
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针（子类边可直接访问全局参数，如权重、约束阈值）
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的核心成员：残差向量（_error）、顶点列表（_vertices）
    using g2o::BaseMultiEdge<D, E>::_error;
    using g2o::BaseMultiEdge<D, E>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐（支持Eigen类型残差）
};

}  // namespace teb_local_planner
