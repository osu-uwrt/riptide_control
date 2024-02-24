cmake_minimum_required(VERSION 3.8)
project(riptide_controllers2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# grab c++ includes
include_directories(include/riptide_controllers2)

# #Build executable
# add_executable(controller src/riptide_controllers2/main.cpp)

# #Define a variable with all dependencies
# set(deps 
#   rclcpp
#   riptide_msgs2
#   geometry_msgs
#   sensor_msgs
#   tf2_geometry_msgs
#   rclpy
#   geometry_msgs
#   control_toolbox
#   tf2
#   tf2_ros
# )

# # Link deps
# ament_target_dependencies(controller ${deps})

# #Install executables
# install(TARGETS controller
#   DESTINATION lib/${PROJECT_NAME}
# )

add_library(motion_profiler SHARED src/riptide_controllers/main.cpp)

install(TARGETS motion_profiler
  DESTINATION lib)


# yoink python programs from src directory
file(GLOB src_py_files RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_controllers/*.py)
install(PROGRAMS ${src_py_files}
  DESTINATION lib/${PROJECT_NAME})

# yoink python programs from models directory
file(GLOB model_py_files RELATIVE ${PROJECT_SOURCE_DIR} models/*.py)
install(PROGRAMS ${model_py_files}
  DESTINATION lib/${PROJECT_NAME})

# yoink python programs from test directory
file(GLOB test_py_files RELATIVE ${PROJECT_SOURCE_DIR} test/riptide_controllers/*.py)
install(PROGRAMS ${test_py_files}
  DESTINATION lib/${PROJECT_NAME})

#Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
