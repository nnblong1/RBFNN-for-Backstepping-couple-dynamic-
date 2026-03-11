cmake_minimum_required(VERSION 3.8)
project(uam_controller)

# ── Chuẩn C++17 ──
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2 -march=native)  # Tối ưu cho CPU hiện tại
endif()

# ── Phụ thuộc ──
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Executable C++: Backstepping + RBFNN ──
add_executable(uam_backstepping_rbfnn_node
  src/uam_backstepping_rbfnn_node.cpp
)

target_include_directories(uam_backstepping_rbfnn_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(uam_backstepping_rbfnn_node
  rclcpp
  px4_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# ── Cài đặt ──
install(TARGETS uam_backstepping_rbfnn_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

# ── Python scripts ──
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/lstm_feedforward_node.py
  scripts/arm_dynamics_node.py
  scripts/train_lstm_offline.py
  scripts/arm_gazebo_command_node.py
  scripts/arm_trajectory_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
