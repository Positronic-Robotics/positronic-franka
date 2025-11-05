cmake_minimum_required(VERSION 3.18)

project(positronic_franka LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/pybind11/CMakeLists.txt")
  message(FATAL_ERROR "Vendored pybind11 not found. Run: git submodule update --init --recursive")
endif()
set(PYBIND11_FINDPYTHON NEW)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/pybind11)
include(FetchContent)
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

set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
FetchContent_Declare(osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v0.6.3
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(osqp)

pybind11_add_module(_franka
    src/bindings.cpp
    src/robot.hpp
)

target_include_directories(_franka PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${osqp_SOURCE_DIR}/include
    ${osqp_BINARY_DIR}/include
)
target_link_libraries(_franka PRIVATE Franka::Franka Eigen3::Eigen ruckig::ruckig osqpstatic)

add_executable(ik_verifier
    tests/ik_verifier.cpp
    src/robot.hpp
)
target_include_directories(ik_verifier PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${osqp_SOURCE_DIR}/include
    ${osqp_BINARY_DIR}/include
)
target_link_libraries(ik_verifier PRIVATE Franka::Franka Eigen3::Eigen ruckig::ruckig osqpstatic)

install(TARGETS _franka
        LIBRARY DESTINATION "positronic_franka"
        RUNTIME DESTINATION "positronic_franka"
        ARCHIVE DESTINATION "positronic_franka")

# Install package __init__.py
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/python/positronic_franka/__init__.py"
        DESTINATION "positronic_franka")
