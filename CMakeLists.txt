cmake_minimum_required(VERSION 3.8)
project(latency_chk)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(latency_chk_proto REQUIRED)
find_package(absl REQUIRED CONFIG)

# include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

###################################################################33
# build executables
set(BUILD_AS_COMPONENT True)

# latency_chk_server
if(BUILD_AS_COMPONENT)
  add_library(latency_chk_server SHARED src/latency_chk_server.cpp)
  rclcpp_components_register_nodes(latency_chk_server "latency_chk::SystemHealthCheckServerImpl")
  target_compile_options(latency_chk_server PUBLIC -DBUILD_AS_COMPONENT)
else()
  add_executable(latency_chk_server src/latency_chk_server.cpp)
endif()
target_link_libraries(latency_chk_server 
  absl::flags
  absl::flags_parse
  absl::strings
)
ament_target_dependencies(latency_chk_server rclcpp rclcpp_components std_msgs latency_chk_proto)

# latency_chk_client
if(BUILD_AS_COMPONENT)
  add_library(latency_chk_client SHARED src/latency_chk_client.cpp)
  rclcpp_components_register_nodes(latency_chk_client "latency_chk::SystemHealthCheckClientImpl")
  target_compile_options(latency_chk_client PUBLIC -DBUILD_AS_COMPONENT)
else()
  add_executable(latency_chk_client src/latency_chk_client.cpp)
endif()
ament_target_dependencies(latency_chk_client rclcpp rclcpp_components std_msgs latency_chk_proto)

# latency_chk_echo
if(BUILD_AS_COMPONENT)
  add_library(latency_chk_echo SHARED src/latency_chk_echo.cpp)
  rclcpp_components_register_nodes(latency_chk_echo "latency_chk::EchoServerImpl")
  target_compile_options(latency_chk_echo PUBLIC -DBUILD_AS_COMPONENT)
else()
  add_executable(latency_chk_echo src/latency_chk_echo.cpp)
endif()
ament_target_dependencies(latency_chk_echo rclcpp rclcpp_components std_msgs latency_chk_proto)


###################################################################33
# install
if(BUILD_AS_COMPONENT)
  install(
    TARGETS 
      latency_chk_server 
      latency_chk_client 
      latency_chk_echo 
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  install(
    TARGETS 
      latency_chk_server 
      latency_chk_client 
      latency_chk_echo 
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
