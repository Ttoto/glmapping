cmake_minimum_required(VERSION 2.8.3)
project(ccmapping)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED )
find_package (yaml-cpp REQUIRED )
find_package (PCL REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    sensor_msgs
    std_msgs
    geometry_msgs
    message_generation
    message_filters
    tf
    )

add_message_files(
    FILES
    local2global.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/"
    )

## Declare a C++ library
add_library(ccmapping
    src/nodelet_globalmap.cpp
    src/nodelet_localmap.cpp
    src/cc_local_map.cpp
    src/cc_agrid.cpp
    src/cc_rviz_vis.cpp
    )

add_dependencies(ccmapping
    ccmapping_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(ccmapping
    ${catkin_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    )

#independent modules
#1 euroc_publisher publish path
#add_executable(vo_repub_rec
#    src/independ_modules/vo_repub_rec.cpp)
#target_link_libraries(vo_repub_rec
#    ${catkin_LIBRARIES})

#2 output to files
#add_executable(w2files
#    src/independ_modules/w2files.cpp)
#target_link_libraries(w2files
#    ${catkin_LIBRARIES})
