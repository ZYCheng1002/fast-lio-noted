#include <common_lib.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>

#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); };

/// *************IMU Process and undistortion
class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();

  ~ImuProcess();

  ///@brief 重置
  void Reset();

  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr& lastimu);

  void set_extrinsic(const V3D& transl, const M3D& rot);

  void set_extrinsic(const V3D& transl);

  void set_extrinsic(const MD(4, 4) & T);

  void set_gyr_cov(const V3D& scaler);

  void set_acc_cov(const V3D& scaler);

  void set_gyr_bias_cov(const V3D& b_g);

  void set_acc_bias_cov(const V3D& b_a);

  Eigen::Matrix<double, 12, 12> Q;

  void Process(const MeasureGroup& meas,
               esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
               PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;  /// 首帧lidar数据的时间

 private:
  ///@brief 初始化零偏等
  void IMU_init(const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);

  ///@brief 根据eskf反向传播进行运动补偿
  void UndistortPcl(const MeasureGroup& meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                    PointCloudXYZI& pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;         /// 运动补偿的点云
  sensor_msgs::ImuConstPtr last_imu_;      /// 上一帧数据的最后一个imu数据
  deque<sensor_msgs::ImuConstPtr> v_imu_;  /// todo:好像没用到
  vector<Pose6D> IMUpose;                  /// 存储带有时间戳的imu pose
  vector<M3D> v_rot_pcl_;                  /// todo: 好像没用到
  M3D Lidar_R_wrt_IMU;                     /// 外参Rot
  V3D Lidar_T_wrt_IMU;                     /// 外参Trans
  V3D mean_acc;                            /// 加速度计均值
  V3D mean_gyr;                            /// 陀螺仪均值
  V3D angvel_last;                         /// fixme
  V3D acc_s_last;                          /// fixme
  double start_timestamp_;                 /// todo: 好像没用到
  double last_lidar_end_time_;             /// 上帧lidar的结束时间
  int init_iter_num = 1;                   /// imu初始化中的imu帧数
  bool b_first_frame_ = true;              /// imu初始化的首帧标志位
  bool imu_need_init_ = true;              /// imu初始化标志位
};

ImuProcess::ImuProcess() : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T) {
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D& transl) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D& transl, const M3D& rot) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D& scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D& scaler) { cov_acc_scale = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMU_init(const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N) {
  /// 1. 初始化重力、陀螺仪偏差、加速度计和陀螺仪协方差
  /// 2. 将加速度测量值归一化为单位重力
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto& imu_acc = meas.imu.front()->linear_acceleration;
    const auto& gyr_acc = meas.imu.front()->angular_velocity;
    /// 首帧初始化一下加速度计和零偏的均值
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto& imu : meas.imu) {
    const auto& imu_acc = imu->linear_acceleration;
    const auto& gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    /// 加速度计和陀螺仪的均值
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;
    /// 加速度计和陀螺仪的协方差
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
    N++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);  /// 重力的单位向量*重力大小

  init_state.bg = mean_gyr;                   /// 陀螺仪的零偏
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;  /// 外参Trans
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;  /// 外参Rot
  kf_state.change_x(init_state);              /// 更新状态

  /// 获取协方差并初始化
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
}

void ImuProcess::UndistortPcl(const MeasureGroup& meas,
                              esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                              PointCloudXYZI& pcl_out) {
  auto v_imu = meas.imu;
  /// 将上一measure的最后一帧imu放到当前measure的最前面
  v_imu.push_front(last_imu_);
  const double& imu_beg_time = v_imu.front()->header.stamp.toSec();  /// 此measure的首个imu时间(上帧最后一个)
  const double& imu_end_time = v_imu.back()->header.stamp.toSec();   /// 此measure的最后imu时间
  const double& pcl_beg_time = meas.lidar_beg_time;                  /// 此measure的最后首个点时间
  const double& pcl_end_time = meas.lidar_end_time;                  /// 此measure的最后一个点时间

  /// 根据点时间对点进行排序
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  /// 存储此measure的开始pose
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(
      set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /// imu predict过程用的到相关变量
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto&& head = *(it_imu);
    auto&& tail = *(it_imu + 1);
    /// 过滤时间戳小于上一帧的lidar的imu数据
    if (tail->header.stamp.toSec() < last_lidar_end_time_) continue;

    /// 中值平滑滤波
    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm();  // - state_inout.ba;

    if (head->header.stamp.toSec() < last_lidar_end_time_) {
      /// 两帧imu刚好在上阵lidar前后,取时间后面的
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
    } else {
      /// 正常时间戳
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    /// 预测
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    /// 获取状态量
    imu_state = kf_state.get_x();
    /// 使用最新的零偏进行更新
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_s_last[i] += imu_state.grav[i];
    }
    /// 计算imu时间和首个点的差值
    double&& offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    /// 保存每个imu pose
    IMUpose.push_back(
        set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /// 1. 最后一个点时间大于imu列队的时间,按照最后一个imu数据继续predict
  /// 2. 最后一个点时间小于imu列队的时间,反向δt进行predict
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state = kf_state.get_x();
  /// 保存当前最后时间用以后续measure
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /// 反向传播对点云进行运动补偿
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    /// 获取位姿
    R_imu << MAT_FROM_ARRAY(head->rot);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);

    for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
           imu_state.offset_T_L_I);  // not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup& meas,
                         esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
                         PointCloudXYZI::Ptr cur_pcl_un_) {
  double t1, t2, t3;
  t1 = omp_get_wtime();

  if (meas.imu.empty()) {
    return;
  };
  ROS_ASSERT(meas.lidar != nullptr);

  /// 首次IMU初始化,初始化完成后下帧进行lidar相关操作
  if (imu_need_init_) {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    /// imu初始化后的last_imu用当前数据的最后一帧代替
    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    /// imu大于10帧完成初始化
    if (init_iter_num > MAX_INI_COUNT) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);  /// 看起来没用上
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }
    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();

  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
