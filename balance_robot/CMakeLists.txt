cmake_minimum_required(VERSION 3.5)
project(balance_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(navio_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(balance_robot_msgs REQUIRED)

# add dlib
# FIXME: This could be done more colcon like
add_subdirectory(../../dlib dlib_build)

# link it to an executable via
# target_link_libraries(executable dlib::dlib)

add_library(${PROJECT_NAME} SHARED
  "src/pid.cpp"
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

add_executable(balance
  src/balance.cpp
  src/pid.cpp
)

target_include_directories(balance PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${navio_vendor_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(balance
  rclcpp
  navio_vendor
  sensor_msgs
  balance_robot_msgs
)

add_executable(orientation
  src/mono_ahrs.cpp
)

ament_target_dependencies(orientation
  rclcpp
  navio_vendor
  sensor_msgs
  balance_robot_msgs
)

add_executable(pid_controller
  src/balance_pid_controller.cpp
  src/pid.cpp
)

ament_target_dependencies(pid_controller
  rclcpp
  navio_vendor
  sensor_msgs
  balance_robot_msgs
)

target_include_directories(pid_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${navio_vendor_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

add_executable(lqr_controller
  src/balance_lqr_controller.cpp
  src/pid.cpp # Should not be necessary ..
)

ament_target_dependencies(lqr_controller
  rclcpp
  navio_vendor
  sensor_msgs
  balance_robot_msgs
)

target_include_directories(lqr_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${navio_vendor_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

add_executable(mpc_controller
  src/balance_mpc_controller.cpp
)

ament_target_dependencies(mpc_controller
  rclcpp
  navio_vendor
  sensor_msgs
  balance_robot_msgs
)

target_include_directories(mpc_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${navio_vendor_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(mpc_controller dlib::dlib)

ament_export_dependencies(rclcpp navio_vendor)
# ament_export_interfaces(export_balance HAS_LIBRARY_TARGET)

install(TARGETS
  balance
  orientation
  pid_controller
  lqr_controller
  mpc_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
