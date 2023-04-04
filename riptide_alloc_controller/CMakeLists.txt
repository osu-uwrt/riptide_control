cmake_minimum_required(VERSION 3.8)
project(riptide_alloc_controller)
include(ExternalProject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(riptide_msgs2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(SIMULINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/riptide_controller_simulink)
set(SIMULINK_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/simulink_generated_files)
set(SIMULINK_SO_NAME Porportional_Follower_Test.so)

add_library(simulink_generated_library SHARED IMPORTED)

if (NOT EXISTS ${SIMULINK_INSTALL_DIR})
  ExternalProject_Add(generated_simulink_library
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/simulink_tmp
    SOURCE_DIR ${SIMULINK_DIR}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  add_dependencies(simulink_generated_library generated_simulink_library)
endif()

if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
  set(SIMULINK_LIB ${SIMULINK_INSTALL_DIR}/bin/arm/${SIMULINK_SO_NAME})
else()
  set(SIMULINK_LIB ${SIMULINK_INSTALL_DIR}/bin/x86/${SIMULINK_SO_NAME})
endif()

set(SIMULINK_INCLUDE ${SIMULINK_INSTALL_DIR}/include)

set_target_properties(simulink_generated_library PROPERTIES
  IMPORTED_LOCATION "${SIMULINK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${SIMULINK_INCLUDE}"
)

add_executable(controller src/controller.cpp src/allocated_controller.cpp)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${SIMULINK_INCLUDE}>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(controller simulink_generated_library)

ament_target_dependencies(controller
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  riptide_msgs2
  nav_msgs
  yaml_cpp_vendor
)

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${SIMULINK_LIB} 
  DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
