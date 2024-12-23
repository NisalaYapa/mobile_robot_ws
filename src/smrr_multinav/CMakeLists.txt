cmake_minimum_required(VERSION 3.8)
project(smrr_multinav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install (
  DIRECTORY config bt_xml launch
  DESTINATION share/${PROJECT_NAME}
)

set(BEHAVIOR_SOURCES
  src/multinav_behaviors.cpp
)

set(TARGET_DEPENDS
    rclcpp
    rclcpp_action
    nav2_msgs
    behaviortree_cpp_v3
    yaml-cpp
    tf2
    tf2_geometry_msgs
)

include_directories(include/${PROJECT_NAME})

add_executable(multinav src/multinav.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(multinav ${TARGET_DEPENDS})
target_link_libraries(multinav ${YAML_CPP_LIBRARIES})

install(
  TARGETS multinav
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
