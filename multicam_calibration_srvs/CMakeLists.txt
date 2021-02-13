cmake_minimum_required(VERSION 3.5.1)
project(multicam_calibration_srvs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_CXX_STANDARD 14 CACHE STRING "c++ standard")
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ParameterCmd.srv"
  )
ament_export_dependencies(rosidl_default_runtime)

ament_package()
