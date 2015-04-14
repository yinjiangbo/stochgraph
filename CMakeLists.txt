cmake_minimum_required(VERSION 2.8)
project(StochGraph)

# Enables testing framework.
include(CTest)

set(SRC
  stoch_graph.cpp
  stoch_edge.cpp
  sumo_util.cpp
  velocity_map.cpp
  local_frame.cpp)

add_library(stoch_graph SHARED ${SRC})

install_targets(/lib stoch_graph)
install_files(/include/stoch_graph .hpp)

# Adds includes for external libraries.
include_directories(/usr/include/glibmm-2.4/)
include_directories(/usr/lib/x86_64-linux-gnu/glibmm-2.4/include/)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/x86_64-linux-gnu/glib-2.0/include/)
include_directories(/usr/include/libxml++-2.6/)
include_directories(/usr/lib/libxml++-2.6/include/)
include_directories(/usr/include/eigen3/)

# Links external libraries.
set(EXTERNAL_LIBS
  /usr/lib/libxml++-2.6.so
  /usr/lib/x86_64-linux-gnu/libglibmm-2.4.so
  /usr/local/lib/libroadie.so
  /usr/lib/libboost_system.so
  /usr/lib/libproj.so
  /usr/lib/libprotobuf.so
  # /usr/lib/libboost_iostreams.so
  # /usr/lib/libboost_filesystem.so
  #  /usr/local/lib/libwheelie.so
  # /usr/lib/x86_64-linux-gnu/libglibmm-2.4.so
  # /usr/lib/x86_64-linux-gnu/libGLEW.so
  # /usr/lib/x86_64-linux-gnu/libGLU.so
  # /usr/lib/x86_64-linux-gnu/mesa/libGL.so
  # /usr/lib/x86_64-linux-gnu/libfltk.so
  # /usr/lib/x86_64-linux-gnu/libfltk_gl.so
)
target_link_libraries(stoch_graph ${EXTERNAL_LIBS})

add_executable(StochGraphUnitTests stoch_graph_unit_tests.cpp)
target_link_libraries(StochGraphUnitTests /usr/src/gtest/libgtest.a)
target_link_libraries(StochGraphUnitTests stoch_graph)

find_package(Threads)
target_link_libraries(StochGraphUnitTests ${CMAKE_THREAD_LIBS_INIT})

add_test(StochGraphUnitTests StochGraphUnitTests)

add_executable(StochEdgeUnitTests stoch_edge_unit_tests.cpp)
target_link_libraries(StochEdgeUnitTests /usr/src/gtest/libgtest.a)
target_link_libraries(StochEdgeUnitTests stoch_graph)

find_package(Threads)
target_link_libraries(StochEdgeUnitTests ${CMAKE_THREAD_LIBS_INIT})

add_test(StochEdgeUnitTests StochEdgeUnitTests)

add_executable(SumoUtilUnitTests sumo_util_unit_tests.cpp)
target_link_libraries(SumoUtilUnitTests /usr/src/gtest/libgtest.a)
target_link_libraries(SumoUtilUnitTests stoch_graph)

find_package(Threads)
target_link_libraries(SumoUtilUnitTests ${CMAKE_THREAD_LIBS_INIT})

add_test(SumoUtilUnitTests SumoUtilUnitTests)

add_executable(LocalFrameUnitTests local_frame_unit_tests.cpp)
target_link_libraries(LocalFrameUnitTests /usr/src/gtest/libgtest.a)
target_link_libraries(LocalFrameUnitTests stoch_graph)

find_package(Threads)
target_link_libraries(LocalFrameUnitTests ${CMAKE_THREAD_LIBS_INIT})

add_test(LocalFrameUnitTests LocalFrameUnitTests)

# Turns on the debugging information.
# TODO This should have an option to turn off.
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb -O0 -Wfatal-errors")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wfatal-errors")

# This turns on the C++11 options.
# TODO This should be set globally.
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_STD_CXX11)
    check_cxx_compiler_flag(--std=c++0x SUPPORTS_STD_CXX01)
    if(SUPPORTS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++11")
    elseif(SUPPORTS_STD_CXX01)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --std=c++0x")
    else()
        message(ERROR "Compiler does not support --std=c++11 or --std=c++0x.")
    endif()
endif()