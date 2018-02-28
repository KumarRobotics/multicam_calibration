cmake_minimum_required(VERSION 3.1.0)
project(multicam_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -fopenmp -g)

find_package(catkin_simple REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

catkin_simple()

cs_add_library(calibration_nodelet
  src/calibration_nodelet.cpp
  src/multicam_apriltag_detector.cpp
  src/calibrator.cpp
  src/camera_extrinsics.cpp
  src/calibration_data.cpp
  src/get_init_pose.cpp
  src/camera_intrinsics.cpp
)
target_link_libraries(calibration_nodelet
  ${CATKIN_LIBRARIES}
  ${CERES_LIBRARIES})

cs_add_library(calibration_test_nodelet
  src/calibration_test_nodelet.cpp
  src/multicam_apriltag_detector.cpp
  src/camera_extrinsics.cpp
  src/calibration_data.cpp
  src/get_init_pose.cpp
  src/camera_intrinsics.cpp)
target_link_libraries(calibration_test_nodelet
  ${CATKIN_LIBRARIES}
  ${CERES_LIBRARIES})

cs_add_executable(test_rectify_node
  src/test_rectify_node.cpp
  src/test_rectify.cpp)
target_link_libraries(test_rectify_node
  ${CATKIN_LIBRARIES})

cs_install()
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_export()
