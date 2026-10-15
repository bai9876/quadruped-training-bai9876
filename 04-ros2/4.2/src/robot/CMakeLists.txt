cmake_minimum_required(VERSION 3.8)
project(quadruped_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp sensor_msgs std_msgs my_robot_interfaces)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/simulation_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulation_node
)

install(PROGRAMS scripts/gamepad_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gamepad_node
)

install(DIRECTORY launch meshes
  DESTINATION share/${PROJECT_NAME}
)
install(FILES black_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
