// How to do initialization with pointclouds
// try using ICP 

// store the first pointcloud of the bagfile -> perform ICP with that -> get an initial estimate of current pose -> then refine it with GPS

#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/registration/icp.h>

int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tarCloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/deepak/IIITD/test/refClouds/1461675988.921382000.pcd", *refCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/deepak/IIITD/test/tarClouds/1461675989.717598000.pcd", *tarCloud);

    // store the reference pointcloud in a folder
    // take the target pointcloud live
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(*refCloud);
    icp.setInputTarget(*tarCloud);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout<<"ICP has converged: "<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<std::endl;
    std::cout<<icp.getFinalTransformation()<<std::endl;

    return 0;

}