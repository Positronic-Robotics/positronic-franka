cmake_minimum_required(VERSION 3.15)

project(positronic_franka LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/pybind11/CMakeLists.txt")
  message(FATAL_ERROR "Vendored pybind11 not found. Run: git submodule update --init --recursive")
endif()
set(PYBIND11_FINDPYTHON NEW)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/pybind11)
find_package(Franka CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(RUCKIG_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/ruckig")
if (EXISTS "${RUCKIG_VENDOR_DIR}")
  add_subdirectory(${RUCKIG_VENDOR_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ruckig)
  if (TARGET ruckig)
    set_target_properties(ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()
else()
  message(FATAL_ERROR "Ruckig not found. Expected at '${CMAKE_CURRENT_SOURCE_DIR}/external/ruckig'.")
endif()

pybind11_add_module(_franka
    src/bindings.cpp
    src/robot.hpp
)

target_include_directories(_franka PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(_franka PRIVATE Franka::Franka Eigen3::Eigen ruckig::ruckig)

install(TARGETS _franka
        LIBRARY DESTINATION "positronic/drivers/roboarm"
        RUNTIME DESTINATION "positronic/drivers/roboarm"
        ARCHIVE DESTINATION "positronic/drivers/roboarm")
