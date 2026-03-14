cmake_minimum_required(VERSION 3.8)
project(alfan_walking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(Eigen3 REQUIRED)

# Definisikan node utama
add_executable(walking_node src/walking_node.cpp)

# --- INJEKSI DIREKTORI INCLUDE STANDAR ROS 2 ---
target_include_directories(walking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link library standar ROS 2
ament_target_dependencies(walking_node
  rclcpp
  std_msgs
  trac_ik_lib
)

# Link library matematika Eigen3 (Harus menggunakan target_link_libraries)
target_link_libraries(walking_node Eigen3::Eigen)

install(TARGETS
  walking_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()