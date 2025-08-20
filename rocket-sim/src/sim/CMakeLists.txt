cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vtvl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sim_dynamics src/sim_dynamics.cpp)
ament_target_dependencies(sim_dynamics rclcpp std_msgs vtvl_msgs geometry_msgs)
target_include_directories(sim_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_dynamics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  sim_dynamics
  DESTINATION lib/${PROJECT_NAME}
)

# --- Add PID Altitude Controller ---
add_executable(pid_alt_controller src/pid_alt_controller.cpp)
ament_target_dependencies(pid_alt_controller rclcpp std_msgs vtvl_msgs)

install(TARGETS
  pid_alt_controller
  DESTINATION lib/${PROJECT_NAME}
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
