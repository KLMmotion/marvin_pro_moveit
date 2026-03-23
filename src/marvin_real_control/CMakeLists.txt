cmake_minimum_required(VERSION 3.5)
project(marvin_real_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  src/${PROJECT_NAME}/marvin_real_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marvin_real_controller
)

install(PROGRAMS
  src/${PROJECT_NAME}/marvin_moveit_bridge_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME marvin_moveit_bridge_demo
)

install(PROGRAMS
  src/${PROJECT_NAME}/jointstate_to_marvin_cmd.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jointstate_to_marvin_cmd
)

install(PROGRAMS
  src/${PROJECT_NAME}/trajectory_to_marvin_cmd.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_to_marvin_cmd
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
