cmake_minimum_required(VERSION 3.8)
project(nav_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
option(NAV_BRIDGE_BUILD_X30 "Build the legacy X30 node" ON)
if(NAV_BRIDGE_BUILD_X30)
  find_package(rviz_2d_overlay_msgs REQUIRED)
endif()
find_package(rcl_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# D1 Max SDK is shipped as architecture-specific prebuilt libraries.  Keep the
# X30-only build usable on machines where the SDK is not installed, while
# allowing ARM64 deployments to enable the backend explicitly.
option(NAV_BRIDGE_BUILD_D1_MAX "Build the D1 Max RobotBackend" OFF)
set(D1_MAX_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/robot_sdk"
    CACHE PATH "Vendored D1 Max RobotSDK root")

# Nav bridge node executable
if(NAV_BRIDGE_BUILD_X30)
add_executable(x30_nav_bridge_node
  src/x30/x30_nav_bridge_node.cpp
  src/x30/action_executor.cpp
  src/x30/udp_transport.cpp
  src/x30/x30_nav_bridge.cpp
)

target_include_directories(x30_nav_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(x30_nav_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rviz_2d_overlay_msgs
  rcl_interfaces
)

install(TARGETS x30_nav_bridge_node
  DESTINATION lib/${PROJECT_NAME})
endif()

if(NAV_BRIDGE_BUILD_D1_MAX)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(D1_MAX_SDK_ARCH aarch64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(D1_MAX_SDK_ARCH x86_64)
  else()
    message(FATAL_ERROR "Unsupported D1 Max SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  set(D1_MAX_SDK_LIBRARY "${D1_MAX_SDK_ROOT}/lib/${D1_MAX_SDK_ARCH}/librobot_sdk.so.0.2.1")
  if(NOT EXISTS "${D1_MAX_SDK_LIBRARY}")
    message(FATAL_ERROR "D1 Max SDK library not found: ${D1_MAX_SDK_LIBRARY}")
  endif()

  add_library(nav_bridge_d1_max_backend STATIC src/d1_max/d1_max_backend.cpp)
  target_include_directories(nav_bridge_d1_max_backend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    "${D1_MAX_SDK_ROOT}/include")
  target_link_libraries(nav_bridge_d1_max_backend PUBLIC "${D1_MAX_SDK_LIBRARY}" Threads::Threads)
  target_compile_features(nav_bridge_d1_max_backend PUBLIC cxx_std_17)
  set_target_properties(nav_bridge_d1_max_backend PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    BUILD_RPATH "${D1_MAX_SDK_ROOT}/lib/${D1_MAX_SDK_ARCH}")
  install(TARGETS nav_bridge_d1_max_backend
    ARCHIVE DESTINATION lib)

  add_executable(d1_max_nav_bridge_node src/d1_max/d1_max_nav_bridge_node.cpp)
  target_include_directories(d1_max_nav_bridge_node PRIVATE
    "${D1_MAX_SDK_ROOT}/include")
  ament_target_dependencies(d1_max_nav_bridge_node
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs rcl_interfaces)
  target_link_libraries(d1_max_nav_bridge_node nav_bridge_d1_max_backend)
  set_target_properties(d1_max_nav_bridge_node PROPERTIES
    BUILD_RPATH "${D1_MAX_SDK_ROOT}/lib/${D1_MAX_SDK_ARCH}"
    INSTALL_RPATH "$ORIGIN/..")
  install(TARGETS d1_max_nav_bridge_node
    DESTINATION lib/${PROJECT_NAME})
  install(FILES "${D1_MAX_SDK_LIBRARY}"
    DESTINATION lib
    RENAME librobot_sdk.so.0.2.1)
endif()

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
