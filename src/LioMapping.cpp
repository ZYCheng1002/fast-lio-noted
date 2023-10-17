//
// Created by czy on 2023/10/16.
//
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "LidarMapping.h"
bool flg_exit = false;
ros::Publisher pubOdomAftMapped;
ros::Publisher pubLaserCloudFull;
ros::Publisher pubLaserCloudFull_body;
ros::Publisher pubLaserCloudMap;

void SigHandle(int sig) {
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
}

void pubOdometry(const PoseWithTime& pose) {
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = ros::Time().fromSec(pose.timestamp);  // ros::Time().fromSec(lidar_end_time);
  auto& p = odomAftMapped.pose;
  p.pose.position.x = pose.position[0];
  p.pose.position.y = pose.position[1];
  p.pose.position.z = pose.position[2];
  p.pose.orientation.x = pose.quat.x();
  p.pose.orientation.y = pose.quat.y();
  p.pose.orientation.z = pose.quat.z();
  p.pose.orientation.w = pose.quat.w();
  pubOdomAftMapped.publish(odomAftMapped);
  for (int i = 0; i < 36; i++) {
    odomAftMapped.pose.covariance[i] = pose.pose_cov(i);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(
      odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void pubPointCloud(const CloudWithTime& cloud) {
    sensor_msgs::PointCloud2 laserCloudmsg_w;
    pcl::toROSMsg(*cloud.cloud_w, laserCloudmsg_w);
    laserCloudmsg_w.header.stamp = ros::Time().fromSec(cloud.timestamp);
    laserCloudmsg_w.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg_w);
    sensor_msgs::PointCloud2 laserCloudmsg_b;
    pcl::toROSMsg(*cloud.cloud_b, laserCloudmsg_b);
    laserCloudmsg_b.header.stamp = ros::Time().fromSec(cloud.timestamp);
    laserCloudmsg_b.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg_b);
}

void pubCloudMap(CloudWithTime cloud) {
  sensor_msgs::PointCloud2 laserCloudMap;
  pcl::toROSMsg(*cloud.cloud_w, laserCloudMap);
  laserCloudMap.header.stamp = ros::Time().fromSec(cloud.timestamp);
  laserCloudMap.header.frame_id = "camera_init";
  pubLaserCloudMap.publish(laserCloudMap);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);

  LioMapping lio_mapping(nh);
  std::thread lio_run = std::thread([&]() { lio_mapping.run(); });

  signal(SIGINT, SigHandle);
  ros::Rate rate(20);
  while (ros::ok()) {
    if (flg_exit) {
      lio_mapping.exit();
      break;
    }
    PoseWithTime pose;
    /// 获取位姿
    if (lio_mapping.getOdom(pose)) {
      pubOdometry(pose);
    }
    /// 获取实时点云
    CloudWithTime cloud;
    if (lio_mapping.getPointCloud(cloud)) {
      pubPointCloud(cloud);
      cloud.cloud_b.reset(new PointCloudXYZI);
      cloud.cloud_w.reset(new PointCloudXYZI);
    }
    /// 获取全局点云
    CloudWithTime cloud_map;
    if (lio_mapping.getCloudMap(cloud_map)) {
      pubCloudMap(cloud_map);
      cloud.cloud_w.reset(new PointCloudXYZI);
    }
  }

  return 0;
}