cmake_minimum_required(VERSION 3.8)
project(h1_mujoco_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_h1_model test/test_h1_model.cpp)
  target_link_libraries(test_h1_model mujoco::mujoco)
  target_compile_definitions(test_h1_model PRIVATE
    H1_SCENE="${H1_MODEL_DIR}/scene.xml")
endif()

ament_package()