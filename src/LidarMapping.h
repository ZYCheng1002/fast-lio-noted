//
// Created by czy on 2023/10/16.
//

#ifndef FAST_LIO_LIDARMAPPING_H
#define FAST_LIO_LIDARMAPPING_H

#include <fast_lio/CustomMsg.h>
#include <geometry_msgs/Vector3.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

#include <Eigen/Core>
#include <cmath>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>

#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "preprocess.h"

#define INIT_TIME_LIO (0.1)
#define LASER_POINT_COV_LIO (0.001)
#define MAXN_LIO (720000)

class LioMapping {
 public:
  LioMapping(const ros::NodeHandle& nh) { nh_ = nh; }
  void run();
  void exit() {
    exit_flag = true;
    sig_buffer.notify_all();
  }

  ///@brief ros msg回调函数
  void standardPclCbk(const sensor_msgs::PointCloud2::ConstPtr& msg);

  ///@brief livox msg回调函数
  void livoxPclCbk(const fast_lio::CustomMsg::ConstPtr& msg);

  ///@brief imu回调函数
  void imuCbk(const sensor_msgs::Imu::ConstPtr& msg_in);

  bool getOdom(PoseWithTime& pose);

  bool getPointCloud(CloudWithTime& cloud);

  bool getCloudMap(CloudWithTime& cloud);

 private:
  void memoryInit();

  void rosParamInit();

  inline void dumpLioStateToLog(FILE* fp);

  ///@brief 将点转为世界坐标系
  void pointBodyToWorld(PointType const* const pi, PointType* const po);

  template <typename T>
  void pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po);

  ///@brief 点转到世界坐标系下的imu坐标系
  void rgbPointBodyToWorld(PointType const* const pi, PointType* const po);

  ///@brief 点转到imu坐标系下
  void rgbPointBodyLidarToIMU(PointType const* const pi, PointType* const po);

  void pointsCacheCollect();

  void lasermapFovSegment();

  ///@brief msg的时间同步
  bool syncPackages(MeasureGroup& meas);

  ///@brief ikdtree增量地图
  void mapIncremental();

  void hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

 private:
  ros::NodeHandle nh_;

  /*** Time Log Variables ***/
  double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
  std::vector<double> T1 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot2 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot3 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot4 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot5 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot6 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot7 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot8 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot9 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot10 = std::vector<double>(MAXN_LIO, 0.0);
  std::vector<double> s_plot11 = std::vector<double>(MAXN_LIO, 0.0);
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;
  int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
  bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
  /**************************/
  std::vector<float> res_last;  // [100000] = {0.0}
  float DET_RANGE = 300.0f;
  const float MOV_THRESHOLD = 1.5f;
  double time_diff_lidar_to_imu = 0.0;
  mutex mtx_buffer;
  condition_variable sig_buffer;
  string root_dir = ROOT_DIR;
  string map_file_path, lid_topic, imu_topic;
  double res_mean_last = 0.05, total_residual = 0.0;
  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
  double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
  int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
  int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1,
      pcd_index = 0;
  std::vector<bool> point_selected_surf;
  bool lidar_pushed, flg_first_scan = true, flg_EKF_inited;
  bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
  vector<vector<int>> pointSearchInd_surf;
  vector<BoxPointType> cub_needrm;
  vector<PointVector> Nearest_Points;
  std::vector<double> extrinT = std::vector<double>(3, 0.0);
  std::vector<double> extrinR = std::vector<double>(9, 0.0);
  deque<double> time_buffer;
  deque<PointCloudXYZI::Ptr> lidar_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  PointCloudXYZI::Ptr featsFromMap;
  PointCloudXYZI::Ptr feats_undistort;   /// 去除畸变后的单帧点云
  PointCloudXYZI::Ptr feats_down_body;   /// lidar坐标系下的点云(去除畸变后的单帧)
  PointCloudXYZI::Ptr feats_down_world;  /// 世界坐标系下的点云
  PointCloudXYZI::Ptr normvec;
  PointCloudXYZI::Ptr laserCloudOri;
  PointCloudXYZI::Ptr corr_normvect;
  PointCloudXYZI::Ptr _featsArray;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  std::shared_ptr<KD_TREE<PointType>> ikdtree;
  V3F XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
  V3F XAxisPoint_world = V3F(LIDAR_SP_LEN, 0.0, 0.0);
  V3D euler_cur;
  V3D position_last = V3D(Zero3d);
  V3D Lidar_T_wrt_IMU = V3D(Zero3d);
  M3D Lidar_R_wrt_IMU = M3D(Eye3d);
  /*** EKF inputs and output ***/
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  vect3 pos_lid;
  nav_msgs::Path path;
  geometry_msgs::Quaternion geoQuat;  /// 世界坐标系下imu的姿态
  shared_ptr<Preprocess> p_pre;
  shared_ptr<ImuProcess> p_imu;

  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;
  int process_increments = 0;

  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_save;  /// 全局点云(世界坐标系下的imu坐标系)
  double timediff_lidar_wrt_imu = 0.0;
  bool timediff_set_flg = false;

  double lidar_mean_scantime = 0.0;
  int scan_num = 0;
  bool exit_flag = false;
  std::atomic<bool> odom_update{false};
  std::atomic<bool> cloud_update{false};
  std::atomic<bool> cloud_map_update{false};
};

#endif  // FAST_LIO_LIDARMAPPING_H
