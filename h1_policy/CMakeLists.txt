cmake_minimum_required(VERSION 3.8)
project(h1_policy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ONNXRUNTIME_ROOT "$ENV{HOME}/libs/onnxruntime-linux-x64-1.31.0"
  CACHE PATH "Folder of the ONNX Runtime C++ release")

find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

file(READ "${ONNXRUNTIME_ROOT}/VERSION_NUMBER" ONNXRUNTIME_VERSION)
string(STRIP "${ONNXRUNTIME_VERSION}" ONNXRUNTIME_VERSION)
if(NOT ONNXRUNTIME_VERSION VERSION_EQUAL "1.31.0")
  message(FATAL_ERROR "Expected ONNX Runtime 1.31.0, found ${ONNXRUNTIME_VERSION}")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNX Runtime ${ONNXRUNTIME_VERSION}: ${ONNXRUNTIME_LIBRARY}")

add_executable(policy_node src/policy_node.cpp)
target_link_libraries(policy_node PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS})

install(TARGETS policy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()