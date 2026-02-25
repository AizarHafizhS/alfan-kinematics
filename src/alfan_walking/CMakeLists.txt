cmake_minimum_required(VERSION 3.8)
project(alfan_walking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(alfan_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(simple_pub src/simple_pub.cpp)
ament_target_dependencies(simple_pub rclcpp alfan_msgs sensor_msgs)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface)

target_include_directories(simple_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simple_pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
