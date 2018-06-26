#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sstream>


laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud2 cloud;
sensor_msgs::PointCloud2 cloud_tf;
sensor_msgs::PointCloud2 cloud_map;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_pcl (new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf_pcl (new pcl::PointCloud<pcl::PointXYZ>());
Eigen::Matrix4f transform;
sensor_msgs::LaserScan scan;
float d = 0.265f;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
  scan = *scan_in;
  projector_.projectLaser(*scan_in, cloud);
}

void tfCallback(const nav_msgs::Odometry::ConstPtr& tf_in) {
  std::cout << "got transform" << std::endl;
  float x = tf_in->pose.pose.position.x;
  float y = tf_in->pose.pose.position.y;
  float z = tf_in->pose.pose.position.z;
  float qx = tf_in->pose.pose.orientation.x;
  float qy = tf_in->pose.pose.orientation.y;
  float qz = tf_in->pose.pose.orientation.z;
  float qw = tf_in->pose.pose.orientation.w;
  tf::Matrix3x3 rotation;
  tf::Quaternion quat(qx,qy,qz,qw);
  rotation.setRotation(quat);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      transform(i,j) = rotation[i][j];
    }
  }
  transform(0,3) = x + d*rotation[0][0];
  transform(1,3) = y + d*rotation[1][0];
  transform(2,3) = z;
  transform(3,3) = 1;
  std::cout << "Applying the following transformation:\n";
  for (int i=0; i<4; ++i) {
    std::cout << "[";
    for (int j=0; j<4; ++j) {
      std::cout << std::setw(12) << transform(i,j) << " ";
    }
    std::cout << "]\n";
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_mapping");

  ros::NodeHandle n;
  
  ros::Subscriber laser_sub = n.subscribe("scan", 1000, laserCallback);
  ros::Subscriber tf_sub = n.subscribe("vesc/odom", 1000, tfCallback);
  ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    pcl_ros::transformPointCloud(transform, cloud, cloud_tf);

    pcl::fromROSMsg(cloud_tf, *cloud_tf_pcl);
    *cloud_map_pcl += *cloud_tf_pcl;
    pcl::toROSMsg(*cloud_map_pcl, cloud_map);
    cloud_map.header.frame_id = "base_link";

    pc_pub.publish(cloud_map);
    std::cout << "Map has " << cloud_map_pcl->points.size() << " points.\n";
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

