#include "planner_manager.h"


namespace teb_local_planner
{
    void plannerManager::runOptimization()
    {
        std::cout << "===== TEB Style Optimization Start =====" << std::endl;
        // 1. 创建并添加顶点
        AddVertices();
        // 2. 创建并添加边
        AddObstacleEdges();
        AddViaPointEdges();
        AddVelocityEdgs();
        AddEdgeKinematics();

        // 3. 执行优化
        for(int i = 0; i < cfg_.no_outer_iterations;i++)
        {
           optimizeGraph();
        }
        // 4. 清空图（保留顶点）
        clearGraph();
    }

    boost::shared_ptr<g2o::SparseOptimizer> plannerManager::initOptimizer()
    {
        // 线程安全注册自定义类型
        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&registerG2OTypes, flag);

        // 配置求解器（二维顶点，动态残差维度）
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // 创建求解器
        auto linear_solver = std::make_unique<LinearSolverType>();
        auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        // 初始化优化器
        auto optimizer = boost::make_shared<g2o::SparseOptimizer>();
        optimizer->setAlgorithm(solver);
        optimizer->setVerbose(cfg_.optimization_verbose);  // 从配置读取verbose

        return optimizer;
    }



    // 注册g2o类型（顶点+边）
    void plannerManager::registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("EDGE_obstacle_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeObstacleConstraint>);
        factory->registerType("EDGE_via_point_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeViaPointConstraint>);
        factory->registerType("EDGE_velocity_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeVelocityConstraint>);

    }

    void plannerManager::setpathInfo(std::vector<tools::pathInfo>& path)
    {
        pathPointArr_ = path;
        vertexId_ = 0;
        std::cout << "timedff real s" << std::endl;
        for(int i = 1; i < path.size();i++)
        {
            double dist = fabs(tools::distanceBetweenTwoPoint(path[i],path[i-1]));
            double dt   = dist / cfg_.max_vel;
            double dt2  = 0;
            if(i <  path.size() - 1)
            {
                // double theta1 = atan2(path[i].y-path[i-1].y,path[i].x-path[i-1].x);
                // double theta2 = atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x);
                dt2   = fabs(tools::normalize_theta(path[i].theta - path[i-1].theta))/cfg_.max_vel_theta;
            }
            dt = std::max(dt,dt2);
            // timediff_vec_.push_back(dt);
            vertexTimeDiff* v = new vertexTimeDiff();
            v->setId(vertexId_++);
            v->setFixed(false);
            v->setEstimate(dt);
            std::cout << "dt =" << dt << std::endl;
            timediff_vec_.push_back(v);
            optimizer_->addVertex(v);
        }
        std::cout << "timedff real end" << std::endl;

    }

    void plannerManager::setObstacleInfo(std::vector<tools::obstacleInfo>& obs)
    {
        obstaclePointInfo_ = obs;
    }

    void plannerManager::getPlannerResults(std::vector<tools::pathInfo>& path)
    {
        for(int i = 0; i < pose_vertices_.size();i++)
        {
            Eigen::Vector3d v = pose_vertices_[i]->estimate();
            std::cout << "x = " << std::fixed << std::setprecision(4) << v.x()  << ", " << v.y()<< ", " << v[2] << ")" << std::endl;
            if(i < timediff_vec_.size())
            {
                std::cout << "dt = " << timediff_vec_[i]->estimate() << std::endl;
            }
            tools::pathInfo temp;
            temp.x =  v.x();
            temp.y =  v.y();
            temp.theta = v[2];
            path.push_back(temp);
        }

        // for(int i = 0;i < timediff_vec_.size();i++)
        // {
        //     std::cout << "dt = " << timediff_vec_[i]->estimate() << std::endl;
        // }
    }

    // 添加顶点
    void plannerManager::AddVertices()
    {
        // 清空旧顶点
        for (auto& v : pose_vertices_)
            delete v;
        pose_vertices_.clear();

        for(int i = 0; i < pathPointArr_.size();i++)
        {
            VertexPoint2D* v = new VertexPoint2D();
            v->setId(vertexId_++);
            v->setFixed(false);  // 显式设置为可优化
            v->setEstimate(Eigen::Vector3d(pathPointArr_[i].x, pathPointArr_[i].y,pathPointArr_[i].theta));
            pose_vertices_.push_back(v);
            optimizer_->addVertex(v);
            // optimizer_->setFixed(false);  // 强制设为可优化
        }   
    }

    void plannerManager::AddEdgeKinematics()
    {
        // create edge for satisfiying kinematic constraints
        Eigen::Matrix<double,2,2> information_kinematics;
        information_kinematics.fill(0.0);
        information_kinematics(0, 0) = cfg_.weight_kinematics_nh;
        information_kinematics(1, 1) = cfg_.weight_kinematics_forward_drive;
        int edge_id = 0;
       
        for(int i = 0; i < pathPointArr_.size() - 1;i++)
        {
            EdgeKineticConstraint* edge_kinetic = new EdgeKineticConstraint();
            edge_kinetic->setId(edge_id++);
            edge_kinetic->setVertex(0,pose_vertices_[i]);
            edge_kinetic->setVertex(1,pose_vertices_[i+1]);
            edge_kinetic->setInformation(information_kinematics);
            edge_kinetic->setcfg(&cfg_);
            optimizer_->addEdge(edge_kinetic);
        }

    }

    void plannerManager::AddVelocityEdgs()
    {
        Eigen::Matrix<double,2,2> information;
        information(0,0) = cfg_.weight_max_vel_x;
        information(1,1) = cfg_.weight_max_vel_theta;
        information(0,1) = 0.0;
        information(1,0) = 0.0;
        int edge_id = 0;
        std::cout << " AddVelocityEdgs 1" << std::endl;
        // std::cout << " "
        for(int i = 0; i < pathPointArr_.size() - 1;i++)
        {
        

            EdgeVelocityConstraint* edge_velocity = new EdgeVelocityConstraint();
            edge_velocity->setId(edge_id++);
            edge_velocity->setVertex(0,pose_vertices_[i]);
            edge_velocity->setVertex(1,pose_vertices_[i+1]);
            edge_velocity->setVertex(2,timediff_vec_[i]);

            edge_velocity->setInformation(information);
            edge_velocity->setcfg(&cfg_);
            optimizer_->addEdge(edge_velocity);
        }
        std::cout << " AddVelocityEdgs 2" << std::endl;


    }

    // 添加边
    void plannerManager::AddObstacleEdges()
    {
        Eigen::Matrix<double,1,1> information;
        information.fill(cfg_.obstacle_weight);//mat.fill(n)　将 mat 的所有元素均赋值为 n
        std::cout << "infomaition" << information << std::endl;
        int edge_id = 0;  // 全局递增ID
        for (int i = 0; i < pose_vertices_.size(); ++i)
        {
            // const VertexPoint2D* v1 = static_cast<const VertexPoint2D*>(pose_vertices_[i]);
            // //    double act_dist = (v1->estimate() - v2->estimate()).norm();
            // Eigen::Vector2d point = v1->estimate();
            // std::cout << "point =" << point << std::endl;
            // continue;
            for(int j = 0; j < obstaclePointInfo_.size();j++)
            {
                EdgeObstacleConstraint* e_soft = new EdgeObstacleConstraint();
                 e_soft->setId(edge_id++);
                // std::cout<< " vertex:" << *pose_vertices_[i] << std::endl;
                std::cout << " obs:"<< obstaclePointInfo_[j].x << " ," << obstaclePointInfo_[j].y << std::endl;
                e_soft->setVertex(0, pose_vertices_[i]);  // 关联顶点1
                e_soft->setTebConfig(cfg_);              // 传递配置（基类接口）
                e_soft->setObstcele(obstaclePointInfo_[j],&cfg_);
                // e_soft->setInformation(information);
                e_soft->setInformation(Eigen::Matrix<double,1,1>::Identity());

                optimizer_->addEdge(e_soft);
            }
        }
    }

    int plannerManager::findClosestTrajectoryPose(tools::pathInfo &ref_point,int& idx)
    {
        int n = pose_vertices_.size();
        if (idx < 0 || idx >= n)
            return -1;

        double min_dist_sq = std::numeric_limits<double>::max();
        int min_idx = -1;

        for (int i = idx; i < n; i++)
        {
            VertexPoint2D* v        =  static_cast<VertexPoint2D*>(pose_vertices_[i]);
            Eigen::Vector3d point   =  v->estimate();

            double dist_sq = sqrt(pow(ref_point.x - point[0],2) + pow(ref_point.y - point[1],2));
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                min_idx = i;
            }
        }

        idx = min_idx;

        return min_idx;
    }

    void plannerManager::AddViaPointEdges()
    {
        int start_index = 0;
        int edge_id = 0;
        for(int i = 0; i < pathPointArr_.size();i++)
        {
            int index = findClosestTrajectoryPose(pathPointArr_[i],start_index);

            Eigen::Matrix<double,1,1> information;
            information.fill(cfg_.weight_viapoint);

            std::cout << "index =" << index << std::endl;
 
            EdgeViaPointConstraint* edge_viapoint = new EdgeViaPointConstraint;
            edge_viapoint->setId(edge_id++);
            edge_viapoint->setVertex(0,pose_vertices_[index]);
            edge_viapoint->setInformation(information);
            edge_viapoint->setPathPoint(pathPointArr_[i],&cfg_);
            optimizer_->addEdge(edge_viapoint);
        }
    }

    // 执行优化
    bool plannerManager::optimizeGraph()
    {
        if (pose_vertices_.empty())
        {
            std::cerr << "Error: No vertices to optimize!" << std::endl;
            return false;
        }
        optimizer_->initializeOptimization();

        double chi2_before = optimizer_->chi2();
        int actual_iter = optimizer_->optimize(cfg_.no_inner_iterations);  // 从配置读取最大迭代次数
        double chi2_after = optimizer_->chi2();
        std::cout << "优化前chi2: " << chi2_before << ", 优化后chi2: " << chi2_after << std::endl;

        if (actual_iter == 0)
        {
            std::cerr << "Optimization failed: No iterations executed!" << std::endl;
            return false;
        }
        return true;

    }

    // 打印优化结果
    void plannerManager::printResult()
    {

        std::cout << "\n===== Optimization Result =====" << std::endl;
        for(int i = 0; i < pose_vertices_.size();i++)
        {
            Eigen::Vector3d v = pose_vertices_[i]->estimate();
            std::cout << "x = " << std::fixed << std::setprecision(4) << v.x()  << ", " << v.y() << ")" << std::endl;

        }
        // Eigen::Vector2d v1_res = pose_vertices_[0]->estimate();
        // Eigen::Vector2d v2_res = pose_vertices_[1]->estimate();
        // double final_dist = (v1_res - v2_res).norm();

        // std::cout << "顶点1坐标：(" << std::fixed << std::setprecision(4) << v1_res.x() 
        //           << ", " << v1_res.y() << ")" << std::endl;
        // std::cout << "顶点2坐标：(" << std::fixed << std::setprecision(4) << v2_res.x() 
        //           << ", " << v2_res.y() << ")" << std::endl;
        // std::cout << "实际距离：" << std::fixed << std::setprecision(4) << final_dist << std::endl;
        // std::cout << "期望距离：" << std::fixed << std::setprecision(4) << cfg_.distance_constraint_exp_dist << std::endl;

    }

}
