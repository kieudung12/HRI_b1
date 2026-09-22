cmake_minimum_required(VERSION 3.8)
project(ur3_draw_letter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(draw_letter_d_node src/draw_letter_d_node.cpp)
ament_target_dependencies(draw_letter_d_node
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_letter_geometry test/test_letter_geometry.cpp)
  target_include_directories(test_letter_geometry PRIVATE include)
  ament_target_dependencies(test_letter_geometry geometry_msgs)
  add_test(NAME letter_geometry COMMAND test_letter_geometry)
endif()

install(TARGETS draw_letter_d_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
