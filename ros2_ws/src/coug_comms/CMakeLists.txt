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
  base_comms_parameters
  "params/base_comms_params.yaml"
)

generate_parameter_library(
  auv_comms_parameters
  "params/auv_comms_params.yaml"
)

# --- Component Library ---
ament_auto_add_library(coug_comms_component SHARED
  src/base_comms.cpp
  src/auv_comms.cpp
)

target_include_directories(coug_comms_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(coug_comms_component
  base_comms_parameters
  auv_comms_parameters
)

rclcpp_components_register_nodes(coug_comms_component
  "coug_comms::BaseCommsNode"
  "coug_comms::AuvCommsNode"
)

# --- Executables ---
ament_auto_add_executable(base_comms src/base_comms_node.cpp)
target_link_libraries(base_comms coug_comms_component)

ament_auto_add_executable(auv_comms src/auv_comms_node.cpp)
target_link_libraries(auv_comms coug_comms_component)

# --- Testing ---
if(BUILD_TESTING)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
