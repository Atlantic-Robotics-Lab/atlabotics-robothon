cmake_minimum_required(VERSION 3.8)
project(moveit_interface)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(gripper_srv REQUIRED)


# find_package(moveit_core REQUIRED)

include_directories(include/moveit_interface)

add_executable(moveit_interface 
  src/main.cpp  
  src/moveit_interface.cpp
)

ament_target_dependencies(moveit_interface
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  yaml-cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_task_constructor_core
  gripper_srv
  # moveit_ros_planning      # ✅ Needed for robot_trajectory
  # moveit_core              # ✅ includes trajectory_processing
)

target_link_libraries(moveit_interface yaml-cpp)  # <- also link it directly

target_include_directories(moveit_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_interface
  "moveit_ros_planning_interface"
  # "moveit_core"
  "rclcpp"
)

install(TARGETS moveit_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
