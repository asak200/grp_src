cmake_minimum_required(VERSION 3.8)
project(px4_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(offboard_control_srv src/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)

add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs)

install(TARGETS
  offboard_control_srv
  offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
