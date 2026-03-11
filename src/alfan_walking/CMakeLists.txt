cmake_minimum_required(VERSION 3.8)
project(alfan_walking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)

# Definisikan node utama (otak walking)
add_executable(walking_node src/walking_node.cpp)

# Wajib melakukan link ke library TRAC-IK
ament_target_dependencies(walking_node
  rclcpp
  std_msgs
  trac_ik_lib
)

install(TARGETS
  walking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()