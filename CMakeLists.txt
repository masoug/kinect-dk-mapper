cmake_minimum_required(VERSION 3.12)
project(kinect_dk_mapper)

set(CMAKE_CXX_STANDARD 14)

set(Boost_USE_MULTITHREADED TRUE)
find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(libpointmatcher REQUIRED PATHS extern/libpointmatcher/_install)
include_directories(${libpointmatcher_INCLUDE_DIRS})
message(STATUS "libpointmatcher found, \n\tversion ${libpointmatcher_VERSION}\n\t(include=${libpointmatcher_INCLUDE_DIRS}\n\tlibs=${libpointmatcher_LIBRARIES})")

add_executable(kinect_dk_mapper main.cpp)
target_link_libraries(kinect_dk_mapper
        ${libpointmatcher_LIBRARIES}
        ${Boost_LIBRARIES})