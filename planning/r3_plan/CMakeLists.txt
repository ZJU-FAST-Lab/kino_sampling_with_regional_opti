cmake_minimum_required(VERSION 2.8.3)
project(r3_plan)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  occ_grid
  poly_traj_utils
)

# find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# message(STATUS "--------------------------------------------------------------------- ")
# message(${OMPL_LIBRARY_DIRS})
# message(STATUS "||||||||||||||||||||||||||||||-------------||||||||||||||||||||||||| ")
# message(${OMPL_LIBRARIES})
# message(STATUS "||||||||||||||||||||||||||||||-------------||||||||||||||||||||||||| ")
# message(${OMPL_INCLUDE_DIRS})
# message(STATUS "-------------------------------------------------------------------- ")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES r3_plan
  CATKIN_DEPENDS occ_grid poly_traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/r3_planner.cpp
  src/a_star_search.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
