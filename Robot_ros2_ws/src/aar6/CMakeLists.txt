cmake_minimum_required(VERSION 3.8)
project(aar6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Add executable
add_executable(joint_state_pub src/joint_state_pub.cpp)
ament_target_dependencies(joint_state_pub
  rclcpp
  sensor_msgs
)

install(TARGETS joint_state_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
