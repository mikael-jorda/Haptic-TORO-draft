cmake_minimum_required(VERSION 3.10)

project(haptic_toro_draft)
set(PROJECT_VERSION 0.1.0)

set(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")
if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif()

# set default build to release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# SAI2-URDF
find_package(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})

# SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})

# SAI2-PRIMITIVES
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})

# application
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

set(SAI_LIBRARIES
${SAI2-URDF_LIBRARIES}
${SAI2-PRIMITIVES_LIBRARIES}
${SAI2-MODEL_LIBRARIES}
${SAI2-COMMON_LIBRARIES}
)

add_executable(Haptic-TORO-draft_main src/main.cpp)
target_link_libraries(Haptic-TORO-draft_main ${SAI_LIBRARIES})

add_executable(Test_UDP_receiver src/test_UDP_receiver.cpp)
target_link_libraries(Test_UDP_receiver ${SAI_LIBRARIES})
