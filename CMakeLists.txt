cmake_minimum_required(VERSION 2.8.3)
project(pc_normal_estimation)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
find_package(PCL)
##### OpenMP #####
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(normal_estimation src/normal_estimation.cpp)
target_link_libraries(normal_estimation
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(normal_estimation_omp src/normal_estimation_omp.cpp)
target_link_libraries(normal_estimation_omp
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

#add_executable(pc_normal_estimation src/pc_normal_estimation.cpp)
#target_link_libraries(pc_normal_estimation ${catkin_LIBRARIES})
#add_executable(normal_estimation_pcl_multithreads src/normal_estimation_pcl_multithreads.cpp)
#target_link_libraries(normal_estimation_pcl_multithreads ${catkin_LIBRARIES})
#add_executable(no_viewer src/no_viewer.cpp)
#target_link_libraries(no_viewer ${catkin_LIBRARIES})
#add_executable(normal_estimation_pcl_omp src/normal_estimation_pcl_omp.cpp)
#target_link_libraries(normal_estimation_pcl_omp ${catkin_LIBRARIES})
#add_executable(omp_flexible_search_range src/omp_flexible_search_range.cpp)
#target_link_libraries(omp_flexible_search_range ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
