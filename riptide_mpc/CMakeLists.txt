cmake_minimum_required(VERSION 3.8)
project(riptide_mpc)

if(NOT CMAKE_BUILD_TYPE)
  # The MPC linearizes by finite differences every tick; it needs optimization.
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(mpc_core src/marine_dynamics.cpp src/thruster_dynamics.cpp src/fossen_model.cpp src/box_qp.cpp src/mpc.cpp src/state_estimator.cpp src/motion_profile.cpp src/path_plan.cpp src/identification.cpp)
target_include_directories(mpc_core PUBLIC include)
target_link_libraries(mpc_core Eigen3::Eigen yaml-cpp)
set_target_properties(mpc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The same CAN sender complete_controller uses on the vehicle (ESC boards on can0).
add_library(thruster_can STATIC ../riptide_controllers/models/complete_system/send_thruster_cmd_canbus.c)
target_include_directories(thruster_can PRIVATE ../riptide_controllers/models/complete_system)
set_target_properties(thruster_can PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(mpc_controller src/mpc_controller_node.cpp)
target_link_libraries(mpc_controller mpc_core thruster_can)
ament_target_dependencies(mpc_controller rclcpp rclcpp_action riptide_msgs2 geometry_msgs nav_msgs std_msgs sensor_msgs tf2_ros)

add_executable(model_study src/model_study.cpp)
target_link_libraries(model_study mpc_core)
ament_target_dependencies(model_study ament_index_cpp)

add_executable(pool_identify src/pool_identify_node.cpp)
target_link_libraries(pool_identify mpc_core)
ament_target_dependencies(pool_identify rclcpp riptide_msgs2 geometry_msgs nav_msgs std_msgs std_srvs sensor_msgs ament_index_cpp)

add_executable(identify_fit src/identify_fit.cpp)
target_link_libraries(identify_fit mpc_core)
ament_target_dependencies(identify_fit ament_index_cpp)

add_executable(bag_prediction src/bag_prediction.cpp)
target_link_libraries(bag_prediction mpc_core)
ament_target_dependencies(bag_prediction rclcpp rosbag2_cpp nav_msgs std_msgs ament_index_cpp)

install(TARGETS mpc_controller model_study pool_identify identify_fit bag_prediction DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/thruster_sweep.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mpc test/test_mpc.cpp TIMEOUT 300)
  target_link_libraries(test_mpc mpc_core)
  ament_add_gtest(test_state_estimator test/test_state_estimator.cpp TIMEOUT 300)
  target_link_libraries(test_state_estimator mpc_core)
  ament_add_gtest(test_identification test/test_identification.cpp TIMEOUT 1200)
  target_link_libraries(test_identification mpc_core)
  ament_add_gtest(test_path_plan test/test_path_plan.cpp TIMEOUT 120)
  target_link_libraries(test_path_plan mpc_core)
  foreach(test_target test_mpc test_state_estimator test_identification)
    target_compile_definitions(${test_target} PRIVATE
      # Frozen vehicle + the simulator's plant: robot-side retuning (riptide_descriptions,
      # config/models/talos.yaml) must not change test results.
      TALOS_VEHICLE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/talos_vehicle.yaml"
      TALOS_HYDRO="${CMAKE_CURRENT_SOURCE_DIR}/config/models/talos_sim.yaml"
      TALOS_MODEL_SIM="${CMAKE_CURRENT_SOURCE_DIR}/config/models/talos_sim.yaml")
  endforeach()

  # The dynamics are a copy of the simulator's; when c_simulator is installed, prove
  # the copy still matches it (not a build dependency otherwise).
  find_package(c_simulator QUIET)
  if(c_simulator_FOUND)
    ament_add_gtest(test_dynamics_parity test/test_dynamics_parity.cpp)
    target_link_libraries(test_dynamics_parity mpc_core c_simulator::marine_dynamics)
    target_compile_definitions(test_dynamics_parity PRIVATE
      TALOS_VEHICLE="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/talos_vehicle.yaml"
      TALOS_MODEL_SIM="${CMAKE_CURRENT_SOURCE_DIR}/config/models/talos_sim.yaml")
  else()
    message(STATUS "c_simulator not found: skipping the dynamics parity test")
  endif()
endif()

ament_package()
