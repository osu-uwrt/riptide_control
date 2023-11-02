cmake_minimum_required(VERSION 3.8)
project(riptide_controllers2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(eigen REQUIRED)
# find_package(libceres-dev REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf_transformations REQUIRED)

#find_package(ament_copyright REQUIRED)
#find_package(ament_flake8 REQUIRED)
#find_package(ament_pep257 REQUIRED)
#find_package(python3-pytest REQUIRED)

find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# grab c++ includes
include_directories(include/riptide_controllers2)

#Build executable
add_executable(controller src/riptide_controllers2/main.cpp)

#Define a variable with all dependencies
set(deps 
  rclcpp
  riptide_msgs2
  geometry_msgs
  # eigen
  #libceres-dev
  sensor_msgs
  tf2_geometry_msgs
  #tf_transformations
  #ament_copyright
  #ament_flake8
  #ament_pep257
  #python3-pytest
  rclpy
  geometry_msgs
  control_toolbox
  tf2
  tf2_ros
)

# Link deps
ament_target_dependencies(controller ${deps})

#Install executables
install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME}
)

# yoink python files
file(GLOB py_files RELATIVE ${PROJECT_SOURCE_DIR} src/riptide_controllers2/*.py)
install(PROGRAMS ${py_files}
  DESTINATION lib/${PROJECT_NAME}
)



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