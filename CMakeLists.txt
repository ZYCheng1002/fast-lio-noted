cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()

include(cmake/packages.cmake)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        pcl_ros
        tf
        message_generation
        eigen_conversions
        )


include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        src
        ${CATKIN_DEVEL_PREFIX}/include
)

include_directories(
        ${PROJECT_SOURCE_DIR}/../../devel/include
)

add_message_files(
        FILES
        Pose6D.msg
        CustomMsg.msg
        CustomPoint.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
        DEPENDS EIGEN3 PCL
        INCLUDE_DIRS
)
add_library(${PROJECT_NAME}_timer src/timer/timer.cpp)
add_library(${PROJECT_NAME}_ikd_tree src/ikd-Tree/ikd_Tree.cpp)
add_library(${PROJECT_NAME}_preprocess src/preprocess.cpp)
add_library(${PROJECT_NAME}_lio src/LidarMapping.cpp)

add_executable(lio_mapping src/LioMapping.cpp)
target_link_libraries(lio_mapping
        ${catkin_LIBRARIES}
        ${third_party}
        ${PROJECT_NAME}_lio
        ${PROJECT_NAME}_ikd_tree
        ${PROJECT_NAME}_preprocess
        ${PROJECT_NAME}_timer
        )

add_executable(fastlio_mapping src/laserMapping.cpp src/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping         ${catkin_LIBRARIES} ${third_party} )