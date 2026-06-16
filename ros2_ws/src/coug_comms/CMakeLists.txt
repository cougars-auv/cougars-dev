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

# --- Parameters ---
generate_parameter_library(
  base_command_relay_parameters
  "params/base_command_relay_params.yaml"
)

generate_parameter_library(
  auv_command_relay_parameters
  "params/auv_command_relay_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(coug_comms_component SHARED
  src/base_command_relay.cpp
  src/auv_command_relay.cpp
)

target_include_directories(coug_comms_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(coug_comms_component
  base_command_relay_parameters
  auv_command_relay_parameters
)

rclcpp_components_register_nodes(coug_comms_component
  "coug_comms::BaseCommandRelayNode"
  "coug_comms::AuvCommandRelayNode"
)

# --- Executables ---
ament_auto_add_executable(base_command_relay src/base_command_relay_node.cpp)
target_link_libraries(base_command_relay coug_comms_component)

ament_auto_add_executable(auv_command_relay src/auv_command_relay_node.cpp)
target_link_libraries(auv_command_relay coug_comms_component)

# --- Testing ---
if(BUILD_TESTING)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
