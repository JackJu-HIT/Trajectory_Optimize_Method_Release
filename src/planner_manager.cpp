#include "planner_manager.h"

namespace teb_local_planner
{
    void plannerManager::runOptimization()
    {
        std::cout << "\n===== TEB Style Optimization Start =====" << std::endl;

        // 前置检查：路径是否有效
        if (pathPointArr_.empty())
        {
            std::cerr << "Error: 路径点为空，终止优化！" << std::endl;
            return;
        }

        // 1. 清空旧数据（避免ID冲突和内存泄漏）
        clearGraph();

        // 2. 按顺序添加顶点（轨迹顶点→时间差顶点）
        AddVertices();
        AddTimeDiffVertices();

        // 3. 添加约束边（确保顶点已全部创建）
        AddObstacleEdges();
        AddViaPointEdges();
        AddVelocityEdgs();
        AddEdgeKinematics();

        // 4. 执行多轮外迭代优化
        for (int i = 0; i < cfg_.no_outer_iterations; i++)
        {
            std::cout << "\n=== 外迭代 " << i + 1 << "/" << cfg_.no_outer_iterations << " ===" << std::endl;
            if (!optimizeGraph())
            {
                std::cerr << "外迭代 " << i + 1 << " 失败，终止优化流程！" << std::endl;
                break;
            }
        }

        // 5. 输出最终结果
        printResult();
        std::cout << "\n===== TEB Style Optimization End =====\n" << std::endl;
    }

    boost::shared_ptr<g2o::SparseOptimizer> plannerManager::initOptimizer()
    {
        // 线程安全注册自定义类型（仅注册一次）
        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&registerG2OTypes, flag);

        // 配置求解器（动态残差维度，兼容所有边类型）
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // 创建求解器（智能指针管理，避免内存泄漏）
        auto linear_solver = std::make_unique<LinearSolverType>();
        auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        // 初始化优化器
        auto optimizer = boost::make_shared<g2o::SparseOptimizer>();
        optimizer->setAlgorithm(solver);
        optimizer->setVerbose(cfg_.optimization_verbose);  // 日志详细度由配置控制

        std::cout << "优化器初始化完成（求解器：Levenberg-Marquardt）" << std::endl;
        return optimizer;
    }

    // 注册g2o自定义类型（顶点+边）
    void plannerManager::registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        if (!factory)
        {
            throw std::runtime_error("无法获取g2o Factory实例，类型注册失败！");
        }

        // 注册顶点类型
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<vertexTimeDiff>);
        
        // 注册边类型
        factory->registerType("EDGE_OBSTACLE_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeObstacleConstraint>);
        factory->registerType("EDGE_VIA_POINT_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeViaPointConstraint>);
        factory->registerType("EDGE_VELOCITY_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeVelocityConstraint>);
        factory->registerType("EDGE_KINETIC_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeKineticConstraint>);

        std::cout << "g2o自定义类型注册完成（2种顶点 + 4种边）" << std::endl;
    }

    // 设置路径信息（仅保存路径，不创建顶点）
    void plannerManager::setpathInfo(std::vector<tools::pathInfo>& path)
    {
        if (path.empty())
        {
            std::cerr << "Warning: 输入路径为空，忽略设置！" << std::endl;
            return;
        }
        pathPointArr_ = path;
        std::cout << "路径信息设置完成，路径点数量：" << pathPointArr_.size() << std::endl;
    }

    // 添加时间差顶点（ID从轨迹顶点数量开始）
    void plannerManager::AddTimeDiffVertices()
    {
        if (pathPointArr_.size() < 2)
        {
            std::cerr << "Warning: 路径点数量<2，无需创建时间差顶点！" << std::endl;
            return;
        }

        std::cout << "\n开始添加时间差顶点（共 " << pathPointArr_.size() - 1 << " 个）" << std::endl;
        for (int i = 1; i < pathPointArr_.size(); i++)
        {
            // 计算初始dt（考虑距离和角度约束，取较大值确保运动学可行）
            double dist = tools::distanceBetweenTwoPoint(pathPointArr_[i], pathPointArr_[i-1]);
            double dt_dist = (dist > 1e-6) ? (dist / cfg_.max_vel) : 0.1;  // 距离对应的dt
            double theta_diff = fabs(tools::normalize_theta(pathPointArr_[i].theta - pathPointArr_[i-1].theta));
            double dt_theta = (theta_diff > 1e-6) ? (theta_diff / cfg_.max_vel_theta) : 0.1;  // 角度对应的dt
            double dt = std::max(dt_dist, dt_theta);

            // 检查ID是否已存在（避免重复注册）
            if (optimizer_->vertices().count(vertexId_) > 0)
            {
                std::cerr << "Warning: 时间差顶点ID " << vertexId_ << " 已存在，跳过并更新ID！" << std::endl;
                vertexId_++;
                continue;
            }

            // 创建时间差顶点
            vertexTimeDiff* v = new vertexTimeDiff();
            v->setId(vertexId_++);
            v->setFixed(false);       // 可优化
            // v->setActive(true);       // 激活顶点（旧版g2o必需）
            v->setEstimate(dt);       // 初始值非0

            // 添加到容器和优化器
            timediff_vec_.push_back(v);
            optimizer_->addVertex(v);

            // 日志：验证ID和初始值
            std::cout << "时间差顶点 ID: " << v->id() << ", 初始dt: " << std::fixed << std::setprecision(4) << dt << std::endl;
        }
    }

    // 设置障碍物信息
    void plannerManager::setObstacleInfo(std::vector<tools::obstacleInfo>& obs)
    {
        obstaclePointInfo_ = obs;
        std::cout << "障碍物信息设置完成，障碍物数量：" << obstaclePointInfo_.size() << std::endl;
    }

    // 获取优化结果
    void plannerManager::getPlannerResults(std::vector<tools::pathInfo>& path)
    {
        path.clear();
        if (pose_vertices_.empty())
        {
            std::cerr << "Error: 轨迹顶点为空，无法获取优化结果！" << std::endl;
            return;
        }

        std::cout << "\n===== 优化结果输出（共 " << pose_vertices_.size() << " 个轨迹点）=====" << std::endl;
        for (int i = 0; i < pose_vertices_.size(); i++)
        {
            Eigen::Vector3d optimized_val = pose_vertices_[i]->estimate();
            std::cout << "轨迹点 " << i << ": x=" << std::fixed << std::setprecision(4) 
                      << optimized_val.x() << ", y=" << optimized_val.y() 
                      << ", theta=" << optimized_val.z() << std::endl;

            // 输出对应的时间差
            if (i < timediff_vec_.size())
            {
                std::cout << "       dt " << i << ": " << std::fixed << std::setprecision(4) << timediff_vec_[i]->estimate() << std::endl;
            }

            // 存入结果路径
            tools::pathInfo temp;
            temp.x = optimized_val.x();
            temp.y = optimized_val.y();
            temp.theta = optimized_val.z();
            path.push_back(temp);
        }
    }

    // 添加轨迹顶点（ID从0开始）
    void plannerManager::AddVertices()
    {
        vertexId_ = 0;  // 重置ID计数器，确保从0开始
        // 释放旧顶点内存
        for (auto& v : pose_vertices_)
            delete v;
        pose_vertices_.clear();

        if (pathPointArr_.empty())
        {
            std::cerr << "Error: 路径点为空，无法创建轨迹顶点！" << std::endl;
            return;
        }

        std::cout << "\n开始添加轨迹顶点（共 " << pathPointArr_.size() << " 个）" << std::endl;
        for (int i = 0; i < pathPointArr_.size(); i++)
        {
            // 检查ID是否已存在（避免重复注册）
            if (optimizer_->vertices().count(vertexId_) > 0)
            {
                std::cerr << "Warning: 轨迹顶点ID " << vertexId_ << " 已存在，跳过并更新ID！" << std::endl;
                vertexId_++;
                continue;
            }

            // 创建轨迹顶点
            VertexPoint2D* v = new VertexPoint2D();
            v->setId(vertexId_++);
            v->setFixed(false);       // 可优化
            // v->setActive(true);       // 激活顶点（旧版g2o必需）
            v->setEstimate(Eigen::Vector3d(
                pathPointArr_[i].x,
                pathPointArr_[i].y,
                pathPointArr_[i].theta
            ));

            // 添加到容器和优化器
            pose_vertices_.push_back(v);
            optimizer_->addVertex(v);

            // 日志：验证ID和初始值
            Eigen::Vector3d init_val = v->estimate();
            std::cout << "轨迹顶点 ID: " << v->id() << ", 初始值: x=" << std::fixed << std::setprecision(4)
                      << init_val.x() << ", y=" << init_val.y() << ", theta=" << init_val.z() << std::endl;
        }
    }

    // 添加运动学约束边
    void plannerManager::AddEdgeKinematics()
    {
        if (pose_vertices_.size() < 2)
        {
            std::cerr << "Warning: 轨迹顶点数量<2，无法添加运动学约束边！" << std::endl;
            return;
        }

        // 配置约束权重
        Eigen::Matrix<double, 2, 2> information_kinematics;
        information_kinematics << cfg_.weight_kinematics_nh, 0,
                                  0, cfg_.weight_kinematics_forward_drive;

        std::cout << "\n添加运动学约束边（共 " << pose_vertices_.size() - 1 << " 条）" << std::endl;
        int edge_id = 0;
        for (int i = 0; i < pose_vertices_.size() - 1; i++)
        {
            EdgeKineticConstraint* edge = new EdgeKineticConstraint();
            edge->setId(edge_id++);
            edge->setVertex(0, pose_vertices_[i]);
            edge->setVertex(1, pose_vertices_[i+1]);
            edge->setInformation(information_kinematics);
            edge->setcfg(&cfg_);
            optimizer_->addEdge(edge);
        }
    }

    // 添加速度约束边
    void plannerManager::AddVelocityEdgs()
    {
        if (pose_vertices_.size() < 2 || timediff_vec_.empty())
        {
            std::cerr << "Warning: 轨迹顶点不足或时间差顶点为空，无法添加速度约束边！" << std::endl;
            return;
        }

        // 配置约束权重
        Eigen::Matrix<double, 2, 2> information;
        information << cfg_.weight_max_vel_x, 0,
                      0, cfg_.weight_max_vel_theta;

        std::cout << "\n添加速度约束边（共 " << timediff_vec_.size() << " 条）" << std::endl;
        int edge_id = 0;
        // 用时间差顶点数量控制循环，避免越界
        for (int i = 0; i < timediff_vec_.size(); i++)
        {
            EdgeVelocityConstraint* edge = new EdgeVelocityConstraint();
            edge->setId(edge_id++);
            edge->setVertex(0, pose_vertices_[i]);
            edge->setVertex(1, pose_vertices_[i+1]);
            edge->setVertex(2, timediff_vec_[i]);
            edge->setInformation(information);
            edge->setcfg(&cfg_);
            optimizer_->addEdge(edge);
        }
        std::cout << "速度约束边添加完成" << std::endl;
    }

    // 添加障碍物约束边
    void plannerManager::AddObstacleEdges()
    {
        if (pose_vertices_.empty() || obstaclePointInfo_.empty())
        {
            std::cerr << "Warning: 轨迹顶点为空或无障碍物，无法添加障碍物约束边！" << std::endl;
            return;
        }

        // 配置约束权重（使用配置文件中的参数）
        Eigen::Matrix<double, 1, 1> information;
        information.fill(cfg_.obstacle_weight);
        std::cout << "\n添加障碍物约束边（轨迹点 " << pose_vertices_.size() << " 个 × 障碍物 " 
                  << obstaclePointInfo_.size() << " 个）" << std::endl;

        int edge_id = 0;
        for (int i = 0; i < pose_vertices_.size(); i++)
        {
            // 检查顶点是否在优化器中，避免无效关联
            if (optimizer_->vertices().count(pose_vertices_[i]->id()) == 0)
            {
                std::cerr << "Warning: 轨迹顶点ID " << pose_vertices_[i]->id() << " 不在优化器中，跳过！" << std::endl;
                continue;
            }

            for (int j = 0; j < obstaclePointInfo_.size(); j++)
            {
                EdgeObstacleConstraint* edge = new EdgeObstacleConstraint();
                edge->setId(edge_id++);
                edge->setVertex(0, pose_vertices_[i]);
                edge->setTebConfig(cfg_);
                edge->setObstcele(obstaclePointInfo_[j], &cfg_);
                edge->setInformation(information);  // 使用配置的权重，确保障碍物约束生效
                optimizer_->addEdge(edge);
            }
        }
        std::cout << "障碍物约束边添加完成，共 " << edge_id << " 条" << std::endl;
    }

    // 查找最近的轨迹顶点（辅助途经点约束）
    int plannerManager::findClosestTrajectoryPose(tools::pathInfo &ref_point, int& idx)
    {
        int n = pose_vertices_.size();
        // 处理空顶点容器
        if (n == 0)
        {
            std::cerr << "Error: 轨迹顶点为空，无法查找最近点！" << std::endl;
            return -1;
        }
        // 处理无效索引
        if (idx < 0 || idx >= n)
        {
            idx = 0;
            std::cerr << "Warning: 查找索引无效，重置为0！" << std::endl;
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        int min_idx = idx;

        // 计算平方距离（无需开方，提高效率且不影响比较）
        for (int i = idx; i < n; i++)
        {
            Eigen::Vector3d point = pose_vertices_[i]->estimate();
            double dist_sq = pow(ref_point.x - point.x(), 2) + pow(ref_point.y - point.y(), 2);
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                min_idx = i;
            }
        }

        idx = min_idx;
        return min_idx;
    }

    // 添加途经点约束边
    void plannerManager::AddViaPointEdges()
    {
        if (pose_vertices_.empty() || pathPointArr_.empty())
        {
            std::cerr << "Warning: 轨迹顶点或路径点为空，无法添加途经点约束边！" << std::endl;
            return;
        }

        // 配置约束权重
        Eigen::Matrix<double, 1, 1> information;
        information.fill(cfg_.weight_viapoint);
        std::cout << "\n添加途经点约束边（共 " << pathPointArr_.size() << " 条）" << std::endl;

        int edge_id = 0;
        int start_index = 0;
        for (int i = 0; i < pathPointArr_.size(); i++)
        {
            int index = findClosestTrajectoryPose(pathPointArr_[i], start_index);
            // 处理无效索引，避免越界
            if (index == -1 || index >= pose_vertices_.size())
            {
                std::cerr << "Warning: 途经点 " << i << " 未找到有效轨迹顶点，跳过！" << std::endl;
                continue;
            }

            EdgeViaPointConstraint* edge = new EdgeViaPointConstraint();
            edge->setId(edge_id++);
            edge->setVertex(0, pose_vertices_[index]);
            edge->setInformation(information);
            edge->setPathPoint(pathPointArr_[i], &cfg_);
            optimizer_->addEdge(edge);

            // 更新起始索引，避免重复匹配
            start_index = index + 1;
        }
    }

    // 执行单次优化迭代
    bool plannerManager::optimizeGraph()
    {
        if (pose_vertices_.empty() || !optimizer_)
        {
            std::cerr << "Error: 无轨迹顶点或优化器未初始化！" << std::endl;
            return false;
        }

        // 显式传入所有可优化顶点（避免旧版g2o自动筛选失败）
        g2o::HyperGraph::VertexSet vertex_set;
        for (auto* v : pose_vertices_) vertex_set.insert(v);
        for (auto* v : timediff_vec_) vertex_set.insert(v);

        // 初始化优化
        bool init_success = optimizer_->initializeOptimization(vertex_set);
        std::cout << "优化初始化 " << (init_success ? "成功" : "失败") 
                  << "，可优化顶点数: " << vertex_set.size() << std::endl;

        if (!init_success)
        {
            std::cerr << "Error: 优化初始化失败！" << std::endl;
            return false;
        }

        // 执行优化迭代
        double chi2_before = optimizer_->chi2();
        int actual_iter = optimizer_->optimize(cfg_.no_inner_iterations);
        double chi2_after = optimizer_->chi2();

        // 输出优化信息（残差下降说明优化有效）
        std::cout << "优化前chi2: " << std::fixed << std::setprecision(6) << chi2_before 
                  << ", 优化后chi2: " << chi2_after 
                  << "，实际迭代次数: " << actual_iter << std::endl;

        return actual_iter > 0;
    }

    // 打印优化结果
    void plannerManager::printResult()
    {
        std::cout << "\n===== 最终优化结果（轨迹点）=====" << std::endl;
        for (int i = 0; i < pose_vertices_.size(); i++)
        {
            Eigen::Vector3d v = pose_vertices_[i]->estimate();
            std::cout << "轨迹点 " << i << ": x=" << std::fixed << std::setprecision(4) 
                      << v.x() << ", y=" << v.y() << ", theta=" << v.z() << std::endl;
        }
    }

    // 清空图（释放内存+重置状态）
    void plannerManager::clearGraph()
    {
        if (optimizer_)
        {
            // 1. 释放轨迹顶点内存
            for (auto& v : pose_vertices_)
            {
                if (v)
                {
                    optimizer_->removeVertex(v);  // 从优化器移除
                    delete v;                    // 释放内存
                    v = nullptr;
                }
            }
            pose_vertices_.clear();

            // 2. 释放时间差顶点内存
            for (auto& v : timediff_vec_)
            {
                if (v)
                {
                    optimizer_->removeVertex(v);  // 从优化器移除
                    delete v;                    // 释放内存
                    v = nullptr;
                }
            }
            timediff_vec_.clear();

            // 3. 清空优化器的边和顶点映射表
            optimizer_->clear();

            // 4. 重置ID计数器
            vertexId_ = 0;

            std::cout << "clearGraph: 内存释放完成，ID计数器重置为0" << std::endl;
        }
    }

}  // namespace teb_local_planner