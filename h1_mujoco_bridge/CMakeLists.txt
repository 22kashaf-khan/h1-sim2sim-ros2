cmake_minimum_required(VERSION 3.8)
project(h1_mujoco_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_ROOT "$ENV{HOME}/libs/mujoco-3.15.0"
  CACHE PATH "Folder of the MuJoCo release")
set(H1_MODEL_DIR "$ENV{HOME}/libs/mujoco_menagerie/unitree_h1"
  CACHE PATH "Folder of the MuJoCo Menagerie H1 model")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(MUJOCO_LIBRARY mujoco
  HINTS "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo: ${MUJOCO_LIBRARY}")

add_library(h1_mujoco_bridge_core STATIC src/h1_sim.cpp)
target_include_directories(h1_mujoco_bridge_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(h1_mujoco_bridge_core PUBLIC mujoco::mujoco)

add_executable(mujoco_bridge src/bridge_node.cpp)
target_link_libraries(mujoco_bridge PUBLIC
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} yaml-cpp::yaml-cpp
  h1_mujoco_bridge_core)
set_target_properties(mujoco_bridge PROPERTIES INSTALL_RPATH "${MUJOCO_ROOT}/lib")

install(TARGETS mujoco_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_h1_model test/test_h1_model.cpp)
  target_link_libraries(test_h1_model mujoco::mujoco)
  target_compile_definitions(test_h1_model PRIVATE
    H1_SCENE="${H1_MODEL_DIR}/scene.xml")

  ament_add_gtest(test_h1_sim test/test_h1_sim.cpp)
  target_link_libraries(test_h1_sim h1_mujoco_bridge_core)
  target_compile_definitions(test_h1_sim PRIVATE
    H1_SCENE="${H1_MODEL_DIR}/scene.xml")

  ament_add_gtest(test_h1_pd test/test_h1_pd.cpp)
  target_link_libraries(test_h1_pd h1_mujoco_bridge_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_h1_pd PRIVATE
    H1_SCENE="${H1_MODEL_DIR}/scene.xml"
    DEPLOY_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../h1_policy/policies/h1_flat_baseline/deploy.yaml")
endif()

ament_package()