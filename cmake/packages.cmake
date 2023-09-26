# eigen
find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})
message("Eigen Version: " ${Eigen3_VERSION})
include_directories(${EIGEN3_INCLUDE_DIR})

# pcl
find_package(PCL 1.8 REQUIRED)
message("PCL Version: " ${PCL_VERSION})
include_directories(${PCL_INCLUDE_DIRS})

set(third_party
        ${PCL_LIBRARIES}
        glog gflags
        )