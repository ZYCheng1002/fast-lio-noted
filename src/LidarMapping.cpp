//
// Created by czy on 2023/10/16.
//
#include "LidarMapping.h"

bool flg_exit = false;
void SigHandle(int sig) {
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
}

void LioMapping::run() {
  memoryInit();
  rosParamInit();

  /*** variables definition ***/
  int effect_feat_num = 0, frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
                         aver_time_solve = 0, aver_time_const_H_time = 0;
  bool flg_EKF_converged, EKF_stop_flg = false;

  FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
  HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);
  _featsArray.reset(new PointCloudXYZI());
  point_selected_surf.resize(100000, true);
  res_last.resize(100000, -1000.0f);
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  point_selected_surf.resize(100000, true);
  res_last.resize(100000, -1000.0f);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

  double epsi[23] = {0.001};
  fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(
      get_f,
      df_dx,
      df_dw,
      [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { hShareModel(s, ekfom_data); },
      NUM_MAX_ITERATIONS,
      epsi);

  /*** debug record ***/
  FILE* fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");

  ofstream fout_pre, fout_out, fout_dbg;
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
  fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
  if (fout_pre && fout_out)
    LOG(INFO) << "~~~~" << ROOT_DIR << " file opened";
  else
    LOG(INFO) << "~~~~" << ROOT_DIR << " doesn't exist";

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl =
      p_pre->lidar_type == AVIA
          ? nh_.subscribe<fast_lio::CustomMsg>(
                lid_topic, 200000, [this](const fast_lio::CustomMsg::ConstPtr& msg) { livoxPclCbk(msg); })
          : nh_.subscribe<sensor_msgs::PointCloud2>(
                lid_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr& msg) { standardPclCbk(msg); });
  ros::Subscriber sub_imu = nh_.subscribe<sensor_msgs::Imu>(
      imu_topic, 200000, [this](const sensor_msgs::Imu::ConstPtr& msg) { imuCbk(msg); });
  ros::Publisher pubLaserCloudFull = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pubLaserCloudFull_body = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  ros::Publisher pubLaserCloudEffect = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
  ros::Publisher pubLaserCloudMap = nh_.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
  ros::Publisher pubOdomAftMapped = nh_.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  ros::Publisher pubPath = nh_.advertise<nav_msgs::Path>("/path", 100000);
  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  bool status = ros::ok();

  ///--------------------------------------------------------

  while (status) {
    if (flg_exit) break;
    ros::spinOnce();
    /// measures为一帧lidar和多帧imu
    if (syncPackages(Measures)) {
      /// 首帧初始化
      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        continue;
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

      match_time = 0;
      kdtree_search_time = 0.0;
      solve_time = 0;
      solve_const_H_time = 0;
      svd_time = 0;
      t0 = omp_get_wtime();
      /// imu 前向传播(predict),点云运动补偿
      p_imu->Process(Measures, kf, feats_undistort);
      state_point = kf.get_x();  /// 获取状态量
      pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

      if (feats_undistort->empty() || (feats_undistort == nullptr)) {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }
      /// 通过时间判断是否完成初始化
      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME_LIO ? false : true;
      /// 根据lidar pose获取周围box信息
      lasermapFovSegment();

      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      t1 = omp_get_wtime();
      feats_down_size = feats_down_body->points.size();
      /// 初始化ikdtree
      if (ikdtree->Root_Node == nullptr) {
        if (feats_down_size > 5) {
          ikdtree->set_downsample_param(filter_size_map_min);
          feats_down_world->resize(feats_down_size);
          for (int i = 0; i < feats_down_size; i++) {
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
          }
          ikdtree->Build(feats_down_world->points);
        }
        continue;
      }
      int featsFromMapNum = ikdtree->validnum();
      kdtree_size_st = ikdtree->size();

      /*** ICP and iterated Kalman filter update ***/
      if (feats_down_size < 5) {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
      fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " "
               << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
               << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
               << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

      if (0)  // If you need to see map point, change to "if(1)"
      {
        PointVector().swap(ikdtree->PCL_Storage);
        ikdtree->flatten(ikdtree->Root_Node, ikdtree->PCL_Storage, NOT_RECORD);
        featsFromMap->clear();
        featsFromMap->points = ikdtree->PCL_Storage;
      }

      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);
      int rematch_num = 0;
      bool nearest_search_en = true;  //

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      double t_update_start = omp_get_wtime();
      double solve_H_time = 0;
      /// 滤波器更新
      kf.update_iterated_dyn_share_modified(LASER_POINT_COV_LIO, solve_H_time);
      /// 完成一轮更新,获取状态量
      state_point = kf.get_x();
      euler_cur = SO3ToEuler(state_point.rot);
      pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
      geoQuat.x = state_point.rot.coeffs()[0];
      geoQuat.y = state_point.rot.coeffs()[1];
      geoQuat.z = state_point.rot.coeffs()[2];
      geoQuat.w = state_point.rot.coeffs()[3];

      double t_update_end = omp_get_wtime();

      /// 发布里程计信息
      publishOdometry(pubOdomAftMapped);

      /*** add the feature points to map kdtree ***/
      t3 = omp_get_wtime();
      mapIncremental();
      t5 = omp_get_wtime();

      /******* Publish points *******/
      if (path_en) publishPath(pubPath);
      if (scan_pub_en || pcd_save_en) publishFrameWorld(pubLaserCloudFull);
      if (scan_pub_en && scan_body_pub_en) publishFrameBody(pubLaserCloudFull_body);
      // publish_effect_world(pubLaserCloudEffect);
      // publish_map(pubLaserCloudMap);

      /*** Debug variables ***/
      if (runtime_pos_log) {
        frame_num++;
        kdtree_size_end = ikdtree->size();
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
        aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = kdtree_incremental_time;
        s_plot4[time_log_counter] = kdtree_search_time;
        s_plot5[time_log_counter] = kdtree_delete_counter;
        s_plot6[time_log_counter] = kdtree_delete_time;
        s_plot7[time_log_counter] = kdtree_size_st;
        s_plot8[time_log_counter] = kdtree_size_end;
        s_plot9[time_log_counter] = aver_time_consu;
        s_plot10[time_log_counter] = add_point_size;
        time_log_counter++;
        printf(
            "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  "
            "map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
            t1 - t0,
            aver_time_match,
            aver_time_solve,
            t3 - t1,
            t5 - t3,
            aver_time_consu,
            aver_time_icp,
            aver_time_const_H_time);
        ext_euler = SO3ToEuler(state_point.offset_R_L_I);
        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " "
                 << state_point.pos.transpose() << " " << ext_euler.transpose() << " "
                 << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose() << " "
                 << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " "
                 << feats_undistort->points.size() << endl;
        dumpLioStateToLog(fp);
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. pcd save will largely influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    LOG(INFO) << "current scan saved to /PCD/" << file_name;
    pcl::VoxelGrid<PointType> down;
    down.setLeafSize(0.3f, 0.3f, 0.3f);
    down.setInputCloud(pcl_wait_save);
    down.filter(*pcl_wait_save);
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }

  fout_out.close();
  fout_pre.close();

  if (runtime_pos_log) {
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    FILE* fp2;
    string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    fp2 = fopen(log_dir.c_str(), "w");
    fprintf(fp2,
            "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree "
            "size st, tree size end, add point size, preprocess time\n");
    for (int i = 0; i < time_log_counter; i++) {
      fprintf(fp2,
              "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
              T1[i],
              s_plot[i],
              int(s_plot2[i]),
              s_plot3[i],
              s_plot4[i],
              int(s_plot5[i]),
              s_plot6[i],
              int(s_plot7[i]),
              int(s_plot8[i]),
              int(s_plot10[i]),
              s_plot11[i]);
      t.push_back(T1[i]);
      s_vec.push_back(s_plot9[i]);
      s_vec2.push_back(s_plot3[i] + s_plot6[i]);
      s_vec3.push_back(s_plot4[i]);
      s_vec5.push_back(s_plot[i]);
    }
    fclose(fp2);
  }
}

void LioMapping::memoryInit() {
  featsFromMap.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());   /// 去除畸变后的单帧点云
  feats_down_body.reset(new PointCloudXYZI());   /// lidar坐标系下的点云(去除畸变后的单帧)
  feats_down_world.reset(new PointCloudXYZI());  /// 世界坐标系下的点云
  normvec.reset(new PointCloudXYZI(100000, 1));
  laserCloudOri.reset(new PointCloudXYZI(100000, 1));
  corr_normvect.reset(new PointCloudXYZI(100000, 1));
  pcl_wait_pub.reset(new PointCloudXYZI(500000, 1));
  pcl_wait_save.reset(new PointCloudXYZI());  /// 全局点云(世界坐标系下的imu坐标系)

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  ikdtree.reset(new KD_TREE<PointType>());
}

void LioMapping::rosParamInit() {
  nh_.param<bool>("publish/path_en", path_en, true);
  nh_.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh_.param<bool>("publish/dense_publish_en", dense_pub_en, true);
  nh_.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
  nh_.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
  nh_.param<string>("map_file_path", map_file_path, "");
  nh_.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh_.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh_.param<bool>("common/time_sync_en", time_sync_en, false);
  nh_.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
  nh_.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
  nh_.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh_.param<double>("filter_size_map", filter_size_map_min, 0.5);
  nh_.param<double>("cube_side_length", cube_len, 200);
  nh_.param<float>("mapping/det_range", DET_RANGE, 300.f);
  nh_.param<double>("mapping/fov_degree", fov_deg, 180);
  nh_.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh_.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh_.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh_.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh_.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh_.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh_.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh_.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh_.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh_.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh_.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
  nh_.param<bool>("runtime_pos_log_enable", runtime_pos_log, false);
  nh_.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
  nh_.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh_.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh_.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
  nh_.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
  LOG(INFO) << "p_pre->lidar_type " << p_pre->lidar_type;

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

inline void LioMapping::dumpLioStateToLog(FILE* fp) {
  V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                             // Angle
  fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));     // Pos
  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // omega
  fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));     // Vel
  fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                  // Acc
  fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));        // Bias_g
  fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));        // Bias_a
  fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);  // Bias_a
  fprintf(fp, "\r\n");
  fflush(fp);
}

void LioMapping::pointBodyToWorldIkfom(PointType const* const pi, PointType* const po, state_ikfom& s) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LioMapping::pointBodyToWorld(PointType const* const pi, PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void LioMapping::pointBodyToWorld(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void LioMapping::rgbPointBodyToWorld(PointType const* const pi, PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LioMapping::rgbPointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LioMapping::pointsCacheCollect() {
  PointVector points_history;
  ikdtree->acquire_removed_points(points_history);
  // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

void LioMapping::lasermapFovSegment() {
  cub_needrm.clear();
  kdtree_delete_counter = 0;
  kdtree_delete_time = 0.0;
  pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
  V3D pos_LiD = pos_lid;  ///当前lidar的位置
  /// 初始化Box
  if (!Localmap_Initialized) {
    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }
    Localmap_Initialized = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  /// 计算lidar到box的边缘位置
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
    /// lidar到box距离太近,意味着要移动box
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      need_move = true;
  }
  if (!need_move) return;
  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  /// 计算box边界
  float mov_dist =
      max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
  for (int i = 0; i < 3; i++) {
    tmp_boxpoints = LocalMap_Points;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  LocalMap_Points = New_LocalMap_Points;

  pointsCacheCollect();
  double delete_begin = omp_get_wtime();
  /// 删除
  if (cub_needrm.size() > 0) kdtree_delete_counter = ikdtree->Delete_Point_Boxes(cub_needrm);
  kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void LioMapping::standardPclCbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  mtx_buffer.lock();
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar = msg->header.stamp.toSec();
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  /// 耗时统计
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LioMapping::livoxPclCbk(const fast_lio::CustomMsg::ConstPtr& msg) {
  mtx_buffer.lock();
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  last_timestamp_lidar = msg->header.stamp.toSec();

  if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
      !lidar_buffer.empty()) {
    printf(
        "IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
  }

  if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty()) {
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
  }

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(last_timestamp_lidar);

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LioMapping::imuCbk(const sensor_msgs::Imu::ConstPtr& msg_in) {
  publish_count++;
  // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
  if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
    msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
  }

  double timestamp = msg->header.stamp.toSec();

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LioMapping::syncPackages(MeasureGroup& meas) {
  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  /// 首帧lidar
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if (meas.lidar->points.size() <= 1) {  /// 点数太少
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
      ROS_WARN("Too few input point cloud!\n");
    } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
      lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
      lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
    }
    meas.lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->header.stamp.toSec();  /// imu列队中首帧时间
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    // imu |||||||||||
    // lid           |
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time) break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void LioMapping::mapIncremental() {
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);
  for (int i = 0; i < feats_down_size; i++) {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    /* decide if need add to map */
    if (!Nearest_Points[i].empty() && flg_EKF_inited) {
      const PointVector& points_near = Nearest_Points[i];
      bool need_add = true;
      BoxPointType Box_of_Point;
      PointType downsample_result, mid_point;
      mid_point.x =
          floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.y =
          floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.z =
          floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
          fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
        PointNoNeedDownsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
    } else {
      PointToAdd.push_back(feats_down_world->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  add_point_size = ikdtree->Add_Points(PointToAdd, true);
  ikdtree->Add_Points(PointNoNeedDownsample, false);
  add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time = omp_get_wtime() - st_time;
}

void LioMapping::publishFrameWorld(const ros::Publisher& pubLaserCloudFull) {
  if (scan_pub_en) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      rgbPointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD_LIO;
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      rgbPointBodyToWorld(&feats_undistort->points[i], &laserCloudWorld->points[i]);
    }
    *pcl_wait_save += *laserCloudWorld;

    static int scan_wait_num = 0;
    scan_wait_num++;
    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void LioMapping::publishFrameBody(const ros::Publisher& pubLaserCloudFull_body) {
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    rgbPointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body.publish(laserCloudmsg);
  publish_count -= PUBFRAME_PERIOD_LIO;
}

void LioMapping::publishEffectWorld(const ros::Publisher& pubLaserCloudEffect) {
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
  for (int i = 0; i < effct_feat_num; i++) {
    rgbPointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i]);
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void LioMapping::publishMap(const ros::Publisher& pubLaserCloudMap) {
  sensor_msgs::PointCloud2 laserCloudMap;
  pcl::toROSMsg(*featsFromMap, laserCloudMap);
  laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudMap.header.frame_id = "camera_init";
  pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void LioMapping::setPosestamp(T& out) {
  out.pose.position.x = state_point.pos(0);
  out.pose.position.y = state_point.pos(1);
  out.pose.position.z = state_point.pos(2);
  out.pose.orientation.x = geoQuat.x;
  out.pose.orientation.y = geoQuat.y;
  out.pose.orientation.z = geoQuat.z;
  out.pose.orientation.w = geoQuat.w;
}

void LioMapping::publishOdometry(const ros::Publisher& pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
  setPosestamp(odomAftMapped.pose);
  pubOdomAftMapped.publish(odomAftMapped);
  auto P = kf.get_P();
  for (int i = 0; i < 6; i++) {
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
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

void LioMapping::publishPath(const ros::Publisher pubPath) {
  setPosestamp(msg_body_pose);
  msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";

  /*** if path is too large, the rviz will crash ***/
  static int jjj = 0;
  jjj++;
  if (jjj % 10 == 0) {
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
  }
}

void LioMapping::hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  double match_start = omp_get_wtime();
  laserCloudOri->clear();
  corr_normvect->clear();
  total_residual = 0.0;

  /** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
  ///@note 多线程中的变量全部声明为shared,因为已经开辟过内存,不会引起竞态条件
#pragma omp parallel for default(none) shared(feats_down_size,     \
                                              s,                   \
                                              feats_down_body,     \
                                              feats_down_world,    \
                                              ekfom_data,          \
                                              ikdtree,             \
                                              point_selected_surf, \
                                              Nearest_Points,      \
                                              normvec,             \
                                              res_last)
#endif
  for (int i = 0; i < feats_down_size; i++) {
    PointType& point_body = feats_down_body->points[i];
    PointType& point_world = feats_down_world->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);  /// 转成世界坐标系
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto& points_near = Nearest_Points[i];
    /// 收敛的情况下
    if (ekfom_data.converge) {
      /// 在ikdtree中查找改点的最近邻
      ikdtree->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
      /// 通过点数和距离判断有效性
      point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                               : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                            : true;
    }
    /// 跳过无效点
    if (!point_selected_surf[i]) continue;

    VF(4) pabcd;
    point_selected_surf[i] = false;
    /// 拟合平面,判断有效性
    if (esti_plane(pabcd, points_near, 0.1f)) {
      /// 计算距离
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
      /// 判断残差有效性
      if (s > 0.9) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = abs(pd2);
      }
    }
  }

  effct_feat_num = 0;

  for (int i = 0; i < feats_down_size; i++) {
    /// 计算有效点的残差
    if (point_selected_surf[i]) {
      laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];  /// 原始点
      corr_normvect->points[effct_feat_num] = normvec->points[i];          /// 对应的平面方程
      total_residual += res_last[i];                                       /// 总残差
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    ROS_WARN("No Effective Points! \n");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;
  match_time += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /// 计算雅克比矩阵和观测向量
  ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  /// 论文中的23,观测雅克比
  ekfom_data.h.resize(effct_feat_num);                  /// 观测维度

  for (int i = 0; i < effct_feat_num; i++) {
    const PointType& laser_p = laserCloudOri->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);  /// 获取lidar点的反对称矩阵
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);  /// 获取世界点的反对称矩阵

    const PointType& norm_p = corr_normvect->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);  /// 平面方程系数

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);  /// R^T * u, u为平面参数
    V3D A(point_crossmat * C);            /// p^ * R^T * u
    /// 观测方程中的H,即雅克比
    if (extrinsic_est_en) {                                       /// 外参估计
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // s.rot.conjugate()*norm_vec);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B),
          VEC_FROM_ARRAY(C);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;  /// 观测的残差
  }
  solve_time += omp_get_wtime() - solve_start_;
}
