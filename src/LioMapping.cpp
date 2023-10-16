//
// Created by czy on 2023/10/16.
//
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "LidarMapping.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  LioMapping lio_mapping(nh);
  lio_mapping.run();

  return 0;
}