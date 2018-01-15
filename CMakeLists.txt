cmake_minimum_required(VERSION 2.8.12)
project(multicam_calibration)

add_compile_options(-std=c++14 -Wall -fopenmp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin_simple REQUIRED)
find_package(Ceres REQUIRED)

catkin_simple()

cs_add_library(calibration_nodelet src/calibration_nodelet.cpp src/multicam_apriltag_detector.cpp src/calibrator.cpp src/camera_extrinsics.cpp src/calibration_data.cpp src/get_init_pose.cpp src/camera_intrinsics.cpp src/intrinsic_estimator.cpp)
target_link_libraries(calibration_nodelet ${CATKIN_LIBRARIES} ${CERES_LIBRARIES})

cs_add_library(calibration_test_nodelet src/calibration_test_nodelet.cpp src/multicam_apriltag_detector.cpp src/camera_extrinsics.cpp src/calibration_data.cpp src/get_init_pose.cpp src/camera_intrinsics.cpp)
target_link_libraries(calibration_test_nodelet ${CATKIN_LIBRARIES})

cs_add_executable(test_rectify_node src/test_rectify_node.cpp src/test_rectify.cpp)
target_link_libraries(test_rectify_node ${CATKIN_LIBRARIES})

cs_install()
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_export()
