cmake_minimum_required(VERSION 3.5.1)
project(multicam_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall)

#
# for c++
#
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(multicam_calibration_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tf2_ros REQUIRED)

set(common_ament_dependencies
  "apriltag_ros"
  "apriltag_msgs"
  "cv_bridge"
  "rclcpp"
  )

foreach(pkg ${common_ament_dependencies})
  find_package(${pkg} REQUIRED)
endforeach()
#
# for python
#
find_package(rclpy REQUIRED)


#
# --------- library holding the common modules
#
add_library(${PROJECT_NAME} SHARED
  src/multicam_apriltag_detector.cpp
  src/calibrator.cpp
  src/camera_extrinsics.cpp
  src/calibration_data.cpp
  src/get_init_pose.cpp
  src/camera_intrinsics.cpp
)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${YAML_CPP_LIBRARIES} opencv_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} ${common_ament_dependencies})
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE include ${CERES_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
#
# -------- calibration component
#
add_library(calibration_component SHARED
  src/calibration_component.cpp)
target_link_libraries(calibration_component ${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} multicam_calibration_srvs std_srvs)

target_include_directories(calibration_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE include)
rclcpp_components_register_nodes(calibration_component "multicam_calibration::CalibrationComponent")
#
# ------- calibration node
#
add_executable(calibration_node
  src/calibration_node.cpp)
target_link_libraries(calibration_node calibration_component ${PROJECT_NAME})
target_include_directories(calibration_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE include)
#
# ------- undistort component
#
add_library(undistort_component SHARED
  src/undistort_component.cpp)
target_link_libraries(undistort_component opencv_core opencv_calib3d)
ament_target_dependencies(undistort_component image_transport cv_bridge)
target_include_directories(undistort_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE include)
rclcpp_components_register_nodes(undistort_component "multicam_calibration::UndistortComponent")
#
# -------- calibration test component
#
# Not ready for prime time yet under ROS2! Compiles, but was never run.
#
add_library(calibration_test_component SHARED
  src/calibration_test_component.cpp)
target_link_libraries(calibration_test_component ${PROJECT_NAME})
ament_target_dependencies(calibration_test_component image_transport cv_bridge tf2_ros)
rclcpp_components_register_nodes(
  calibration_test_component "multicam_calibration::CalibrationTestComponent")


install(TARGETS
  ${PROJECT_NAME}
  calibration_component
  calibration_test_component
  undistort_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  INCLUDES include)

# Install python scripts
install(PROGRAMS
  scripts/example_calib_manager.py
  scripts/kalibr_to_camerainfo.py
  DESTINATION lib/${PROJECT_NAME}/)

install(TARGETS
  calibration_node
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  calib
  DESTINATION share/${PROJECT_NAME}/)

ament_export_targets(export_${PROJECT_NAME})

ament_package()
