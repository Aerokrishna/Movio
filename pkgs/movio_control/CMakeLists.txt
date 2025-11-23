cmake_minimum_required(VERSION 3.8)
project(movio_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(movio_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(control scripts/control.cpp)
ament_target_dependencies(control rclcpp movio_interfaces nav_msgs geometry_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  include
  DESTINATION share/&{PROJECT_NAME}/
)

ament_package()
