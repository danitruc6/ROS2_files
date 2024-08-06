cmake_minimum_required(VERSION 3.8...3.15)
project(my_robot_interfaces)

if(POLICY CMP0148) 
  cmake_policy(SET CMP0148 OLD) 
endif()


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/ComputeRectangleArea.srv"
  "msg/LedPanelState.msg"
  "srv/SetLed.srv"
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
  "srv/CatchTurtle.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
