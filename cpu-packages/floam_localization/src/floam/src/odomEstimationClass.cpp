// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/octree/octree_pointcloud.h>
#include <chrono>

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();   
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=1;
    
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
    std::cout<<"Map Size is: "<<laserCloudCornerMap->size()<<std::endl;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in, bool relocalize){
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);

    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();
    // std::cout << "q:\n" << q_w_curr.matrix() << "\nt:\n" << t_w_curr.matrix() << "\n";

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_s =  std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    ROS_DEBUG_STREAM("down sampling: " << elapsed_s.count());
    //ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    
    if(laserCloudCornerMap->points.size()>10 && laserCloudSurfMap->points.size()>50){
        // start = std::chrono::high_resolution_clock::now();
        // since the entire map is initially set as KdInputCloud during initialisation
        // we don't need to recreate
        // kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        // kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
        // end = std::chrono::high_resolution_clock::now();
        // elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // ROS_DEBUG_STREAM("kdTree set: " << elapsed_s.count());
        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            start = std::chrono::high_resolution_clock::now();
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);
            end = std::chrono::high_resolution_clock::now();
            elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);;
            ROS_DEBUG_STREAM("edge and surface cost: " << elapsed_s.count());

            start = std::chrono::high_resolution_clock::now();
            ceres::Solver::Options options;
            options.num_threads = 8;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;

            ceres::Solve(options, &problem, &summary);
            end = std::chrono::high_resolution_clock::now();
            elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);;
            ROS_DEBUG_STREAM("ceres solver: " << elapsed_s.count());

            uint edge_blocks_num = edge_blocks.size();
            uint surface_blocks_num = surface_blocks.size();

            // start = std::chrono::high_resolution_clock::now();
            // std::vector<double> edge_costs, surface_costs;
            // edge_costs.reserve(edge_blocks_num);
            // surface_costs.reserve(surface_blocks_num);
            // double cost;
            // // std::cout << "edge costs: ";
            // for (auto id : edge_blocks){
            //     problem.EvaluateResidualBlock(id, loss_function, &cost, nullptr, nullptr);
            //     edge_costs.emplace_back(cost);
            // }
            // // for(auto elem : edge_costs)
            // //      std::cout << elem << " ";
            // // std::cout << "\nsurface costs: ";
            // for (auto id : surface_blocks){
            //     problem.EvaluateResidualBlock(id, loss_function, &cost, nullptr, nullptr);
            //     surface_costs.emplace_back(cost);
            // }
            // // for(auto elem : surface_costs)
            // //      std::cout << elem << " ";
            // end = std::chrono::high_resolution_clock::now();
            // elapsed_s = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);;
            // ROS_INFO_STREAM("residual time: " << elapsed_s.count());
            
            std::cout << "\nSummary cost:" << summary.final_cost << "\n";
            final_cost = summary.final_cost;
            average_cost = final_cost / (edge_blocks_num + surface_blocks_num);

            if (edge_blocks_num == 0 && surface_blocks_num == 0)
            {
                ROS_ERROR("No edges or surface residuals");
                for (int i = 0 ; i < 6; i++)
                {
                    correct_covariance.diagonal()[i] = INFINITY;
                }
            }
            else
            {
                // ceres::Covariance::Options covar_options;
                // covar_options.num_threads = 8;
                // covar_options.algorithm_type = ceres::DENSE_SVD;
                // ceres::Covariance covariance(covar_options);
                // std::vector<std::pair<const double*, const double*> > covariance_blocks;
                // auto pai = std::make_pair(const_cast<double*>(parameters),const_cast<double*>(parameters));
                // covariance_blocks.push_back(pai);
                // double covariance_rpyxyz[6 * 6];
                // if (covariance.Compute(covariance_blocks, &problem))
                // {
                // covariance.GetCovarianceBlockInTangentSpace(const_cast<double*>(parameters),const_cast<double*>(parameters), covariance_rpyxyz);
                // }
                // // convert covariance_rpyxyz to correct order xyzrpy
                // Eigen::MatrixXd original = Eigen::Map<Eigen::MatrixXd, Eigen::RowMajor>(&covariance_rpyxyz[0], 6, 6);
                correct_covariance.setZero();
                // correct_covariance.topLeftCorner(3, 3) = original.bottomRightCorner(3, 3);
                // correct_covariance.bottomRightCorner(3, 3) = original.topLeftCorner(3, 3);
                for (int i = 0 ; i < 6; i++)
                {
                    correct_covariance.diagonal()[i] = average_cost;
                }
            }
            edge_blocks.clear();
            surface_blocks.clear();
            // edge_costs.clear();
            // surface_costs.clear();
        }
    }
    else
    {
        ROS_WARN("not enough points in map to associate, map error");
    }
    // std::cout << "q:\n" << q_w_curr.matrix() << "\nt:\n" << t_w_curr.matrix() << "\n";
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;

    // add points to map only when the translation difference between the previous and current lidar scan is sufficient
    //if((last_odom.translation() - odom.translation()).norm() > 0.30)
    //addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
    if(!relocalize)
    {
        addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud);
    }

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    //po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const int K = 5;
        kdtreeEdgeMap->nearestKSearch(point_temp, K, pointSearchInd, pointSearchSqDis);
        // for points in range 1.0 sq m, find covariance matrix
        int min_avg = 3;
        int in_range = 0;
        for (int i = 0; i < K; ++i)
        {
            if (pointSearchSqDis[i] < 1.0)
            {
                in_range++;
            }
        }
        if (in_range >= min_avg)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < in_range; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);

                center = center + tmp;
                nearCorners.push_back(tmp);
            }

            center = center / float(K);

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < in_range; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            // eigen values are sorted in increasing order
            // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html#aaf4ed4172a517a4b9f0ab222f629e261
            // for an edge we are requiring largest eigenvalue or variance magnitude must be atleast 3*x larger than other eigenvalues
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                ceres::ResidualBlockId id = problem.AddResidualBlock(cost_function, loss_function, parameters);
                edge_blocks.push_back(id);
                corner_num++;   
            }
            else
            {
                // ROS_WARN("Not really like an edge");
                ;
            }
        }
        else
        {
            // ROS_WARN("Not enough points in 1m radius for each point!");
        }
    }
    
    if(corner_num<20)
    {
        ROS_WARN("Not enough correct points, corner %d", corner_num);
    }
    else
        ROS_DEBUG("Corners available in scan %d", corner_num);

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        const int K = 5;
        kdtreeSurfMap->nearestKSearch(point_temp, K, pointSearchInd, pointSearchSqDis); //find nearest neighbours in the edge map and store distances and indices

        // Ax + By + Cz + D = 0 or ax + by + cz + 1 = 0
        // ax + by + cz = -1
        Eigen::Matrix<double, K, 3> matA0;
        Eigen::Matrix<double, K, 1> matB0 = -1 * Eigen::Matrix<double, K, 1>::Ones();
        int min_avg = 3;
        int in_range = 0;
        for (int i = 0; i < K; ++i)
        {
            if (pointSearchSqDis[i] < 1.0)
            {
                in_range++;
            }
        }
        if (in_range >= min_avg)
        {
            
            for (int j = 0; j < in_range; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < in_range; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                ceres::ResidualBlockId id = problem.AddResidualBlock(cost_function, loss_function, parameters);
                surface_blocks.push_back(id);
                surf_num++;
            }
        }

    }
    if(surf_num<20){
        ROS_WARN("not enough correct points, surface %d", surf_num);
    }
    else
        ROS_DEBUG("Surfaces available in scan %d", surf_num);

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud){


    // keep the feature maps size as 10k
    // if the size crosses 10k -> remove the excess points from the starting

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp); 
    }
    
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }
/*
    int th = 20000;
    
    if(laserCloudCornerMap->size() > th)
    {
        std::cout<<"Edge feature vector size is more than 10000 "<<laserCloudCornerMap->size()-th<<std::endl;
        laserCloudCornerMap->erase(laserCloudCornerMap->begin(), laserCloudCornerMap->begin() + (laserCloudCornerMap->size() - th));
        std::cout<<"Edge feature vector size is (after erasing)"<<laserCloudCornerMap->size()<<std::endl;
    }

    if(laserCloudSurfMap->size() > th)
    {
        laserCloudSurfMap->erase(laserCloudSurfMap->begin() , laserCloudSurfMap->begin() +(laserCloudSurfMap->size() - th));
    }
*/
    std::cout<<"Edge map size is: "<<laserCloudCornerMap->size()<<std::endl;
    
    double x_min = +odom.translation().x()-100;
    double y_min = +odom.translation().y()-100;
    double z_min = +odom.translation().z()-100;
    double x_max = +odom.translation().x()+100;
    double y_max = +odom.translation().y()+100;
    double z_max = +odom.translation().z()+100;
    
    //ROS_DEBUG("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);

}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap){
    
    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass(){

}
