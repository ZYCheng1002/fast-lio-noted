//
// Created by idriver-czy on 2023/10/17.
//

#ifndef FAST_LIO_LIDARMAPPARAM_H
#define FAST_LIO_LIDARMAPPARAM_H

#include "common_lib.h"

struct LioParam {
  bool dense_pub_en = true;    // 是否发布稠密点云
  int NUM_MAX_ITERATIONS = 4;  // ieskf中的迭代次数
  std::string map_file_path = "";
  bool time_sync_en = false;
  double time_diff_lidar_to_imu = 0.0;
  double filter_size_corner_min = 0.5;
  double filter_size_surf_min = 0.5;
  double filter_size_map_min = 0.5;
  double cube_len = 200;
  float DET_RANGE = 300.f;
  double fov_deg = 180;
  double gyr_cov = 0.1;
  double acc_cov = 0.1;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  double blind = 0.01;
  int lidar_type = AVIA;
  int N_SCANS = 16;
  int time_unit = US;
  int SCAN_RATE = 10;
  int point_filter_num = 2;
  bool feature_enabled = false;
  bool runtime_pos_log = false;
  bool extrinsic_est_en = true;  // 是否外参优化
  bool pcd_save_en = false;
  int pcd_save_interval = -1;
  std::vector<double> extrinT = std::vector<double>(3, 0.0);  // 外参t
  std::vector<double> extrinR = std::vector<double>(9, 0.0);  // 外参R
};

#endif  // FAST_LIO_LIDARMAPPARAM_H
