cmake_minimum_required(VERSION 3.8)
project(hw_interface_ssmr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# --- LibSerial ---
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h)
find_library(LIBSERIAL_LIBRARY NAMES serial)

if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "LibSerial not found. Please install libserial-dev.")
endif()

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIR}
)

# ---- Add the shared library ----
add_library(${PROJECT_NAME} SHARED
  src/mobile_based_hw_interface.cpp
  src/comm_class.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# ---- Export the plugin description ----
pluginlib_export_plugin_description_file(hardware_interface hw_interface_ssmr.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()