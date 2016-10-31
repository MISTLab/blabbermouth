cmake_minimum_required(VERSION 2.8.12)
if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.0)
    message(FATAL "Require gcc-4.x")
endif()

# Set module directory
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(Pthreads REQUIRED)

# Dependency checks
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
find_package(aruco REQUIRED )
#find_package(Bluez REQUIRED)

# Compilation flags
#include_directories(${PTHREADS_INCLUDE_DIR} ${BLUEZ_INCLUDE_DIRS})
include_directories(${PTHREADS_INCLUDE_DIR})
add_definitions(-Wall -std=c++11 ${PTHREADS_DEFINITIONS})

# Target compilation
add_executable(blabbermouth
  bm_datastream.h bm_datastream.c
  bm_aruco2.h bm_aruco2.cpp
  bm_tcp_datastream.h bm_tcp_datastream.c
  bm_dispatcher.h bm_dispatcher.c
  main.cpp)
#  bm_bt_datastream.h bm_bt_datastream.c
target_link_libraries(blabbermouth
  ${aruco_LIBS}
  ${PTHREADS_LIBRARY})
#  ${BLUEZ_LIBRARIES})
