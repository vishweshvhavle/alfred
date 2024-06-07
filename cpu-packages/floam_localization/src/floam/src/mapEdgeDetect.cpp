
//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <chrono>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


//ros lib 
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
lidar::Lidar lidar_param;

double total_time =0;
int total_frame=0;

void laser_processing(){

    mutex_lock.lock();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    std::string mapFileName = "/home/divyat/alive/src/floam/maps/floamMap_Test_12Nov_bag.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFileName, *pointcloud_in);
    mutex_lock.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>);          
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
    end = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsed_seconds = end - start;
    total_frame++;
    float time_temp = elapsed_seconds.count() * 1000;
    total_time+=time_temp;
    //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);  
    *pointcloud_filtered+=*pointcloud_edge;
    *pointcloud_filtered+=*pointcloud_surf;
    // laserProcessing.lidar_param.max_distance=30;
    

    pcl::io::savePCDFileASCII ("filtered.pcd", *pointcloud_filtered);
    pcl::io::savePCDFileASCII ("laser_cloud_edge.pcd", *pointcloud_edge);
    pcl::io::savePCDFileASCII ("laser_cloud_surf.pcd", *pointcloud_surf);
    std::cerr << "Saved data points to pcd." << std::endl;


    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);

}


int main(int argc, char **argv)
{
    int scan_line = 16;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 20.0;
    double min_dis = 2.0;

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    std::string mapFileName = "/home/divyat/alive/src/floam/maps/floamMap_Test_12Nov_bag.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZI>(mapFileName, *pointcloud_in);
    std::cerr << "points in map " << pointcloud_in->points.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>);          
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>);


    laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
    
    pcl::io::savePCDFileASCII ("edge.pcd", *pointcloud_edge);
    pcl::io::savePCDFileASCII ("surf.pcd", *pointcloud_surf);
    std::cerr << "Saved data points to pcd." << std::endl;

    return 0;
}