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

# Add hardware interface dependencies
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)

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

# Install Python modules
install(PROGRAMS
  ${PROJECT_NAME}/nodes/onrobot_driver_node.py
  ${PROJECT_NAME}/hardware/onrobot_gripper_hardware_interface.py
  ${PROJECT_NAME}/hardware/onrobot_gripper_plugin.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install hardware interface plugin description
install(FILES
  hardware/onrobot_gripper_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install resources
install(FILES
  resource/onrobot_driver
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()