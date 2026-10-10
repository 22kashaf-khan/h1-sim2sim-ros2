cmake_minimum_required(VERSION 3.8)
project(h1_policy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(policy_node src/policy_node.cpp)
target_link_libraries(policy_node PUBLIC rclcpp::rclcpp)

install(TARGETS policy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()