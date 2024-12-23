cmake_minimum_required(VERSION 3.8)
project(smrr_interfaces)

# Add compile options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Entities.msg"
  "msg/InferenceResult.msg"
  "msg/Yolov10Inference.msg"
  "msg/Buffer.msg"
  "msg/DataElementFloat.msg"
  "msg/DataElementInt.msg"
  "msg/DataElementString.msg"
  "msg/PrefVelocity.msg"
  "msg/VelocityClassData.msg"
  "action/NavigateToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs geometry_msgs action_msgs)

# Build tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package definition
ament_package()

