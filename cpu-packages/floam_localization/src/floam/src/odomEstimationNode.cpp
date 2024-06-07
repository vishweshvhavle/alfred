// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <math.h>
#include <string.h>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <alive_msgs/StampedFloat64.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

nav_msgs::Odometry laserOdometry;

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserPose;
ros::Publisher pubInitializedPoints;
ros::Publisher pubInitEdge;
ros::Publisher pubInitSurface;
ros::Publisher pubCost;
tf2_ros::Buffer buffer;

pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);

bool is_odom_inited = false;
bool gotFirstPoint = false;

bool relocalize = false;
bool gps_initialization = false;
bool useImuYaw = false;

double total_time = 0;
int total_frame = 0;

bool gotInitialPose = false;
double initialX, initialY = 0;
double initialRoll, initialPitch, initialYaw = 0;
geometry_msgs::Pose initialPose;

/* Subscribe initialPose published from rviz */
void initialPoseHandler(geometry_msgs::PoseWithCovarianceStamped msg)
{
  if (relocalize && !is_odom_inited && !gotInitialPose)
  {
    initialX = msg.pose.pose.position.x;
    initialY = msg.pose.pose.position.y;
    tf::Quaternion initialQ(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(initialQ);
    m.getRPY(initialRoll, initialPitch, initialYaw);
    std::cout << "x: " << initialX << " y: " << initialY << " Yaw: " << initialYaw << std::endl;
    gotInitialPose = true;
  }
}

/** Subscribe to plane features topic published by feature extractor **/
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  mutex_lock.lock();
  pointCloudSurfBuf.push(laserCloudMsg);
  mutex_lock.unlock();
}

/* Subscribe to edge features topic published by feature extractor */
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  mutex_lock.lock();
  pointCloudEdgeBuf.push(laserCloudMsg);
  mutex_lock.unlock();
}

void odom_estimation()
{
  while (1)
  {
    if (!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty())
    {
      // read data
      mutex_lock.lock();
      if (!pointCloudSurfBuf.empty() &&
          (pointCloudSurfBuf.front()->header.stamp.toSec() <
           pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
      {
        pointCloudSurfBuf.pop();
        ROS_WARN("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock.unlock();
        continue;
      }

      if (!pointCloudEdgeBuf.empty() &&
          (pointCloudEdgeBuf.front()->header.stamp.toSec() <
           pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
      {
        pointCloudEdgeBuf.pop();
        ROS_WARN("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock.unlock();
        continue;
      }

      // if time aligned
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
      pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
      ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
      pointCloudEdgeBuf.pop();
      pointCloudSurfBuf.pop();
      mutex_lock.unlock();

      // perform initialization here
      if (is_odom_inited == false)
      {
        /* If performing localization on pre-made maps */
        if (relocalize)
        {
          while (!gotInitialPose)
          {
            ROS_WARN_STREAM_DELAYED_THROTTLE(5, "Waiting for /initialpose");
            ros::spinOnce();
            continue;
          }

          double x = initialX;
          double y = initialY;
          double z = 0;
          // std::cout << " After initilization x: " << initialX << " y: " << initialY << " Yaw: " << initialYaw <<
          // std::endl;

          double min_distance = 3;
          double max_distance = 5;
          double z_min = 10000;
          
          for(int i=0;i<pointcloud_surf_in->points.size();i++){
              double distance = sqrt((pointcloud_surf_in->points[i].x -x)* (pointcloud_surf_in->points[i].x -x) + (pointcloud_surf_in->points[i].y -y)* (pointcloud_surf_in->points[i].y -y));
              if(distance>min_distance && distance<max_distance){
                  if(pointcloud_surf_in->points[i].z<z_min){
                      z_min = pointcloud_surf_in->points[i].z;
                      std::cout<<z_min<<std::endl;
                  }
              }
          }
          z = z_min+1.5;

          odomEstimation.odom.translation().x() = x;
          odomEstimation.odom.translation().y() = y;
          odomEstimation.odom.translation().z() = z;

          odomEstimation.last_odom.translation().x() = x;
          odomEstimation.last_odom.translation().y() = y;
          odomEstimation.last_odom.translation().z() = z;

          Eigen::AngleAxisd rollAngle(initialRoll, Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd yawAngle(initialYaw, Eigen::Vector3d::UnitZ());
          Eigen::AngleAxisd pitchAngle(initialPitch, Eigen::Vector3d::UnitY());

          Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
          q.normalize();

          odomEstimation.odom.rotate(q.matrix());
          odomEstimation.last_odom.rotate(q.matrix());
        }

        odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
        sensor_msgs::PointCloud2 edge_out, surf_out;
        pcl::toROSMsg(*(odomEstimation.laserCloudCornerMap), edge_out);
        pcl::toROSMsg(*(odomEstimation.laserCloudSurfMap), surf_out);
        edge_out.header.stamp = ros::Time::now();
        edge_out.header.frame_id = "map";
        surf_out.header = edge_out.header;
        pubInitEdge.publish(edge_out);
        pubInitSurface.publish(surf_out);
        is_odom_inited = true;
        ROS_INFO("odom inited");
        ros::Duration(0.2).sleep();
      }

      else
      {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        // geometry_msgs::TransformStamped transformStamped;
        // odomEstimation.last_odom = odomEstimation.odom;
        // try
        // {
        //   ROS_INFO("transforming");
        //   transformStamped = buffer.lookupTransform("map", "base_link", ros::Time(0));
        //   odomEstimation.odom = tf2::transformToEigen(transformStamped.transform);
        //   std::cout << "q:\n" << odomEstimation.odom.rotation() << "\nt:\n" << odomEstimation.odom.translation() << "\n";
        // }
        // catch (tf2::TransformException& ex)
        // {
        //   ROS_WARN("%s", ex.what());
        //   continue;
        // }

        odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, relocalize);
        alive_msgs::StampedFloat64 cost_msg;
        cost_msg.data = odomEstimation.final_cost;
        cost_msg.data2 = odomEstimation.average_cost;
        pubCost.publish(cost_msg);
        end = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame++;
        float time_temp = elapsed_seconds.count() * 1000;
        total_time += time_temp;
        ROS_INFO("odom estimation time %f ms", time_temp);
        ROS_INFO("average odom estimation time %f ms and frame count is %d \n \n", total_time / total_frame,
                 total_frame);
      }

      Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
      Eigen::Vector3d t_current = odomEstimation.odom.translation();

      // static tf::TransformBroadcaster br;
      // tf::Transform transform;
      // transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
      // tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
      // transform.setRotation(q);
      // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

      geometry_msgs::PoseStamped carCoord;
      carCoord.header.frame_id = "map";
      carCoord.header.stamp = pointcloud_time;
      carCoord.pose.position.x = t_current.x();
      carCoord.pose.position.y = t_current.y();
      carCoord.pose.position.z = t_current.z();
      carCoord.pose.orientation.x = q_current.x();
      carCoord.pose.orientation.y = q_current.y();
      carCoord.pose.orientation.z = q_current.z();
      carCoord.pose.orientation.w = q_current.w();
      pubLaserPose.publish(carCoord);

      // base_link to map transform will be provided by ekf after fusing necessary estimated

      // try
      // {
      //   buffer.transform(carCoord, carTf, "map", ros::Duration(0.05));
      // }
      // catch (tf2::TransformException& ex)
      // {
      //   throw std::runtime_error(ex.what());
      // }

      // publish odometry
      laserOdometry.header.frame_id = "map";
      laserOdometry.child_frame_id = "base_link";
      laserOdometry.header.stamp = pointcloud_time;
      laserOdometry.pose.pose.orientation = carCoord.pose.orientation;
      laserOdometry.pose.pose.position = carCoord.pose.position;

      // need to deep copy
      auto it = odomEstimation.correct_covariance.data();
      for (unsigned int i = 0; i < laserOdometry.pose.covariance.size(); ++i)
      {
        laserOdometry.pose.covariance[i] = *it;
        ++it;
      }

      pubLaserOdometry.publish(laserOdometry);
    }

    // sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  int scan_line = 64;
  double vertical_angle = 2.0;
  double scan_period = 0.1;
  double max_dis = 60.0;
  double min_dis = 2.0;
  double map_resolution = 0.4;
  double ndt_transform_epsilon = 0;
  double ndt_step_size = 0.0;
  double ndt_resolution = 0.0;
  std::string mapFileName;
  std::string lidarPoseTopic = "/lidarPose";

  nh.getParam("/relocalization", relocalize);
  nh.getParam("/gps_initialization", gps_initialization);
  nh.getParam("/useImuYaw", useImuYaw);

  nh.getParam("/scan_period", scan_period);
  nh.getParam("/vertical_angle", vertical_angle);
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);
  nh.getParam("/map_resolution", map_resolution);
  nh.getParam("/ndt_transform_epsilon", ndt_transform_epsilon);
  nh.getParam("/ndt_step_size", ndt_step_size);
  nh.getParam("/ndt_resolution", ndt_resolution);
  nh.getParam("/mapFileName", mapFileName);
  nh.getParam("/lidarPoseTopic", lidarPoseTopic);

  // initialise odometry covariances
  // TODO use covariances based on ICP quality
  std::vector<double> pose_covar(36);
  nh_private.getParam("pose_covariance", pose_covar);
  auto it_pose = laserOdometry.pose.covariance.begin();
  std::cout << "pose:"
            << "\n";
  for (auto var : pose_covar)
  {
    std::cout << var << " ";
    *it_pose = var;
    ++it_pose;
  }
  std::cout << "\n";
  std::vector<double> twist_covar(36);
  nh_private.getParam("twist_covariance", twist_covar);
  auto it_twist = laserOdometry.twist.covariance.begin();
  std::cout << "twist:"
            << "\n";
  for (auto var : twist_covar)
  {
    std::cout << var << " ";
    *it_twist = var;
    ++it_twist;
  }
  std::cout << "\n";
  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  odomEstimation.init(lidar_param, map_resolution);

  ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 1, velodyneEdgeHandler);
  ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1, velodyneSurfHandler);
  ros::Subscriber initialPosition = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, initialPoseHandler);
  tf2_ros::TransformListener listener(buffer);
  // store the map here
  pcl::io::loadPCDFile<pcl::PointXYZI>(mapFileName, *map);

  pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
  pubLaserPose = nh.advertise<geometry_msgs::PoseStamped>(lidarPoseTopic, 1);

  pubInitEdge = nh.advertise<sensor_msgs::PointCloud2>("/edge_init", 1);
  pubInitSurface = nh.advertise<sensor_msgs::PointCloud2>("/surf_init", 1);
  pubCost = nh.advertise<alive_msgs::StampedFloat64>("/ceres_cost", 1);

  std::thread odom_estimation_process{ odom_estimation };

  ros::spin();

  return 0;
}
