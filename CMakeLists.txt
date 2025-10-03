cmake_minimum_required(VERSION 3.8)
project(onrobot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install node executable
install(PROGRAMS
  scripts/onrobot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules - ADD THESE LINES
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/${PROJECT_NAME}
)

# Install hardware interface files
install(DIRECTORY hardware/
  DESTINATION share/${PROJECT_NAME}/hardware
)

# Install resources
install(FILES
  resource/onrobot_driver
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install hardware interface plugin
install(FILES
  hardware/onrobot_gripper_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()