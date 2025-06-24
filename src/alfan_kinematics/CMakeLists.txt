cmake_minimum_required(VERSION 3.8)
project(alfan_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target called alfan_kinematics
add_library(alfan_kinematics src/alfan_kinematics.cpp)
 
# this line to exports the library
ament_export_targets(alfan_kinematics HAS_LIBRARY_TARGET)
 
# install the include/alfan_kinematics directory to the install/include/alfan_kinematics
install(
  DIRECTORY include/alfan_kinematics

  DESTINATION include
)
 
install(
  TARGETS alfan_kinematics

  EXPORT alfan_kinematics

  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
