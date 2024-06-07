// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <chrono>
#include <iostream>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>


//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"

bool relocalize = false;
bool use_actual_utm_no_offset = false;

int count = 0;
double top_yaw = 0.0;

pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
sensor_msgs::PointCloud2::Ptr mapMsg(new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr tfMapMsg((new sensor_msgs::PointCloud2));

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;
// ros::Time time_stamp;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;
ros::Publisher pubLaserCloudTransformed;
ros::Publisher pubAprioriMap;
tf2_ros::Buffer buffer;

int transformToActualUtmNoOffset(void){
    mapMsg->header.frame_id = "actual_utm_no_offset";
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = buffer.lookupTransform("map", mapMsg->header.frame_id, ros::Time(0));
    } 
    catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    // transformPointCloud (const std::string &target_frame, const geometry_msgs::Transform &net_transform, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out)
    pcl_ros::transformPointCloud("map", transformStamped.transform, *mapMsg, *tfMapMsg);
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*tfMapMsg, pcl_cloud);
    pcl::io::savePCDFile("utm_transformed_map.pcd", pcl_cloud);
    std::cout<< "Saved File"<< std::endl;

    //pointCloudBuf.push(tfMapMsg) for localization to work
    mapMsg = tfMapMsg;
    return 0;
}


void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr laserCloudMsg)
{
    // pubAprioriMap.publish(mapMsg);
    sensor_msgs::PointCloud2::Ptr laserCloudMsgTransformed(new sensor_msgs::PointCloud2);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        // source to base_link transform
        transformStamped = buffer.lookupTransform("base_link", laserCloudMsg->header.frame_id, ros::Time(0), ros::Duration(0.05));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }
    pcl_ros::transformPointCloud("base_link", transformStamped.transform, *laserCloudMsg, *laserCloudMsgTransformed);
    pubLaserCloudTransformed.publish(laserCloudMsgTransformed);
    
    mutex_lock.lock();
    if(!relocalize)
        pointCloudBuf.push(laserCloudMsgTransformed);
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    // std::cout << "time_stamp" << time_stamp << "\n";
    while(1){
        if(relocalize)
        {
            // load 3km map initially
            laserProcessing.lidar_param.max_distance=3000;
            pubAprioriMap.publish(mapMsg);
        }
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";

            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

            relocalize=false;
            laserProcessing.lidar_param.max_distance=30; // threshold till 30meters

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    std::string mapFileName;
    std::string lidarTopic = "/lidar103/velodyne_points";

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/relocalization", relocalize);
    nh.getParam("/mapFileName", mapFileName);
    nh.getParam("/lidarTopic", lidarTopic);
    nh.getParam("/use_actual_utm_no_offset", use_actual_utm_no_offset);
    nh.getParam("/top_yaw", top_yaw);

    std::cout << max_dis << "," << min_dis << "," << scan_line << std::endl;

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1);

    pubLaserCloudTransformed = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_transformed", 1);

    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 1);

    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1); 

    pubAprioriMap = nh.advertise<sensor_msgs::PointCloud2>("/apriori_map", 1);

    if(relocalize)
    {
        pcl::io::loadPCDFile<pcl::PointXYZI>(mapFileName, *map);
        pcl::toROSMsg(*map, *mapMsg);
        mapMsg->header.stamp = ros::Time::now();
        
        if (use_actual_utm_no_offset){
            transformToActualUtmNoOffset();
        }
        else mapMsg->header.frame_id = "map";

        mutex_lock.lock();
        // first pointcloud loaded to buffer
        pointCloudBuf.push(mapMsg);
        mutex_lock.unlock();
    }


    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 1, velodyneHandler);
    tf2_ros::TransformListener listener(buffer);
    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}

