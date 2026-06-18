cmake_minimum_required(VERSION 3.22)
project(coug_comms)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Parameters ---
generate_parameter_library(
  base_cmd_relay_parameters
  "params/base_cmd_relay_params.yaml"
)

generate_parameter_library(
  auv_cmd_relay_parameters
  "params/auv_cmd_relay_params.yaml"
)

generate_parameter_library(
  base_status_relay_parameters
  "params/base_status_relay_params.yaml"
)

generate_parameter_library(
  auv_status_relay_parameters
  "params/auv_status_relay_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(coug_comms_component SHARED
  src/base_cmd_relay.cpp
  src/auv_cmd_relay.cpp
  src/base_status_relay.cpp
  src/auv_status_relay.cpp
)

target_include_directories(coug_comms_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(coug_comms_component
  base_cmd_relay_parameters
  auv_cmd_relay_parameters
  base_status_relay_parameters
  auv_status_relay_parameters
  Eigen3::Eigen
)

rclcpp_components_register_nodes(coug_comms_component
  "coug_comms::BaseCmdRelayNode"
  "coug_comms::AuvCmdRelayNode"
  "coug_comms::BaseStatusRelayNode"
  "coug_comms::AuvStatusRelayNode"
)

# --- Executables ---
ament_auto_add_executable(base_cmd_relay src/base_cmd_relay_node.cpp)
target_link_libraries(base_cmd_relay coug_comms_component)

ament_auto_add_executable(auv_cmd_relay src/auv_cmd_relay_node.cpp)
target_link_libraries(auv_cmd_relay coug_comms_component)

ament_auto_add_executable(base_status_relay src/base_status_relay_node.cpp)
target_link_libraries(base_status_relay coug_comms_component)

ament_auto_add_executable(auv_status_relay src/auv_status_relay_node.cpp)
target_link_libraries(auv_status_relay coug_comms_component)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_status_codec test/test_status_codec.cpp)
  target_include_directories(test_status_codec PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_status_codec Eigen3::Eigen)
  ament_target_dependencies(test_status_codec coug_interfaces geometry_msgs)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
