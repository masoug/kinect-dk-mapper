cmake_minimum_required(VERSION 3.12)
project(kinect_dk_mapper)

set(CMAKE_CXX_STANDARD 14)

set(Boost_USE_MULTITHREADED TRUE)
find_package(Boost COMPONENTS thread filesystem system program_options date_time chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(libpointmatcher REQUIRED PATHS extern/libpointmatcher/_install)
include_directories(${libpointmatcher_INCLUDE_DIRS})
message(STATUS "libpointmatcher found, \n\tversion ${libpointmatcher_VERSION}\n\t(include=${libpointmatcher_INCLUDE_DIRS}\n\tlibs=${libpointmatcher_LIBRARIES})")


set(MAPPING_LIB libkinectdkmapping)
add_library(${MAPPING_LIB}
        KinectDKMapper.cpp)
target_link_libraries(${MAPPING_LIB}
        ${libpointmatcher_LIBRARIES}
        ${Boost_LIBRARIES})


set(MAPPER_BIN kinect-dk-mapper)
add_executable(${MAPPER_BIN} main.cpp SetupMapping.cpp)
target_link_libraries(${MAPPER_BIN} ${MAPPING_LIB})
