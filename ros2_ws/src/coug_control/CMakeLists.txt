# Copyright 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(coug_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(COUG_CONTROL_NODES hsd_mode_converter)

set(COUG_CONTROL_NODE_SOURCES "")
set(COUG_CONTROL_PARAM_LIBRARIES "")
foreach(node IN LISTS COUG_CONTROL_NODES)
  generate_parameter_library(
    ${node}_parameters
    "params/${node}_params.yaml"
  )
  list(APPEND COUG_CONTROL_NODE_SOURCES "src/${node}.cpp")
  list(APPEND COUG_CONTROL_PARAM_LIBRARIES "${node}_parameters")
endforeach()

ament_auto_add_library(coug_control_component SHARED NO_TARGET_LINK_LIBRARIES
  ${COUG_CONTROL_NODE_SOURCES}
)

target_compile_features(coug_control_component PUBLIC cxx_std_17)

target_link_libraries(coug_control_component ${COUG_CONTROL_PARAM_LIBRARIES})

rclcpp_components_register_nodes(coug_control_component
  "coug_control::HsdModeConverterNode"
)

foreach(node IN LISTS COUG_CONTROL_NODES)
  ament_auto_add_executable(${node} NO_TARGET_LINK_LIBRARIES "src/${node}_node.cpp")
  target_link_libraries(${node} coug_control_component)
endforeach()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
