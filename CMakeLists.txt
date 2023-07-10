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
find_package(std_msgs REQUIRED)
find_package(latency_chk_proto REQUIRED)
find_package(absl REQUIRED CONFIG)

# include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# build executables
add_executable(latency_chk_server src/latency_chk_server.cpp)
add_executable(latency_chk_client src/latency_chk_client.cpp)
ament_target_dependencies(latency_chk_server rclcpp std_msgs latency_chk_proto)
ament_target_dependencies(latency_chk_client rclcpp std_msgs latency_chk_proto)
target_link_libraries(latency_chk_server 
  absl::flags
  absl::flags_parse
  absl::strings
)
install(
  TARGETS latency_chk_server latency_chk_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
