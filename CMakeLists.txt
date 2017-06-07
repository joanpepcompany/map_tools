cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(CGAL REQUIRED)
if ( CGAL_FOUND )
  include(${CGAL_USE_FILE})
  include(CGAL_CreateSingleSourceCGALProgram)
  include_directories (BEFORE "../include")
else()
  message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
)
find_package(PCL 1.7 REQUIRED COMPONENTS io registration segmentation visualization recognition)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cgal
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(map_tools src/hello_world)
link_directories(${CGAL_LIBRARIES})

link_libraries(map_tools ${CGAL_LIBRARIES}) 

target_include_directories (map_tools PUBLIC ${CGAL_INCLUDE_DIRS})

target_link_libraries(map_tools PUBLIC ${CGAL_LIBRARIES})
set(CONF_INCLUDE_DIRS "${CGAL_INCLUDE_DIRS}")

add_executable(compute_normals_from_PCD_2_XYZN_file src/compute_normals_from_PCD_2_XYZN_file.cpp)
target_link_libraries(compute_normals_from_PCD_2_XYZN_file ${catkin_LIBRARIES})
