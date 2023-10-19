//
// Created by czy on 2023/10/16.
//
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "LidarMapping.h"
#include "timer/timer.h"
bool flg_exit = false;
ros::Subscriber sub_pcl;
ros::Subscriber sub_imu;
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

struct Param{
  bool path_enable = false;
  bool scan_pub_enable = true;
  bool scan_body_pub_enable = true;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  LioParam lio_param;
  Param mapping_param;
  std::string lid_topic, imu_topic;
  nh.param<bool>("publish/path_en", mapping_param.path_enable, true);
  nh.param<bool>("publish/scan_publish_en", mapping_param.scan_pub_enable, true);
  nh.param<bool>("publish/dense_publish_en", lio_param.dense_pub_en, true);
  nh.param<bool>("publish/scan_bodyframe_pub_en", mapping_param.scan_body_pub_enable, true);
  nh.param<int>("max_iteration", lio_param.NUM_MAX_ITERATIONS, 4);
  nh.param<string>("map_file_path", lio_param.map_file_path, "");
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/time_sync_en", lio_param.time_sync_en, false);
  nh.param<double>("common/time_offset_lidar_to_imu", lio_param.time_diff_lidar_to_imu, 0.0);
  nh.param<double>("filter_size_corner", lio_param.filter_size_corner_min, 0.5);
  nh.param<double>("filter_size_surf", lio_param.filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", lio_param.filter_size_map_min, 0.5);
  nh.param<double>("cube_side_length", lio_param.cube_len, 200);
  nh.param<float>("mapping/det_range", lio_param.DET_RANGE, 300.f);
  nh.param<double>("mapping/fov_degree", lio_param.fov_deg, 180);
  nh.param<double>("mapping/gyr_cov", lio_param.gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", lio_param.acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", lio_param.b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", lio_param.b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", lio_param.blind, 0.01);
  nh.param<int>("preprocess/lidar_type", lio_param.lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", lio_param.N_SCANS, 16);
  nh.param<int>("preprocess/timestamp_unit", lio_param.time_unit, US);
  nh.param<int>("preprocess/scan_rate", lio_param.SCAN_RATE, 10);
  nh.param<int>("point_filter_num", lio_param.point_filter_num, 2);
  nh.param<bool>("feature_extract_enable", lio_param.feature_enabled, false);
  nh.param<bool>("runtime_pos_log_enable", lio_param.runtime_pos_log, false);
  nh.param<bool>("mapping/extrinsic_est_en", lio_param.extrinsic_est_en, true);
  nh.param<bool>("pcd_save/pcd_save_en", lio_param.pcd_save_en, false);
  nh.param<int>("pcd_save/interval", lio_param.pcd_save_interval, -1);
  nh.param<vector<double>>("mapping/extrinsic_T", lio_param.extrinT, vector<double>());
  nh.param<vector<double>>("mapping/extrinsic_R", lio_param.extrinR, vector<double>());
  LOG(INFO) << "p_pre->lidar_type " << lio_param.lidar_type;
  LioMapping lio_mapping(lio_param);
  sub_pcl =
      lio_param.lidar_type == AVIA
          ? nh.subscribe<fast_lio::CustomMsg>(
                lid_topic, 200000, [&](const fast_lio::CustomMsg::ConstPtr& msg) { lio_mapping.livoxPclCbk(msg); })
          : nh.subscribe<sensor_msgs::PointCloud2>(
                lid_topic, 200000, [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                  lio_mapping.standardPclCbk(msg);
                });
  sub_imu = nh.subscribe<sensor_msgs::Imu>(
      imu_topic, 200000, [&](const sensor_msgs::Imu::ConstPtr& msg) { lio_mapping.imuCbk(msg); });
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);

  std::thread lio_run = std::thread([&]() { lio_mapping.run(); });

  signal(SIGINT, SigHandle);
  while (ros::ok()) {
    ros::spinOnce();
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
    if (mapping_param.scan_pub_enable && lio_mapping.getPointCloud(cloud)) {
      pubPointCloud(cloud);
      cloud.cloud_b.reset(new PointCloudXYZI);
      cloud.cloud_w.reset(new PointCloudXYZI);
    }
    /// 获取全局点云
    CloudWithTime cloud_map;
    if (false && lio_mapping.getCloudMap(cloud_map)) {
      pubCloudMap(cloud_map);
      cloud.cloud_w.reset(new PointCloudXYZI);
    }
  }
  Timer::PrintAll();
  return 0;
}