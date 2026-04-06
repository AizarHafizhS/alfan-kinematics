cmake_minimum_required(VERSION 3.8)
project(alfan_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf bioloid DESTINATION share/${PROJECT_NAME})
