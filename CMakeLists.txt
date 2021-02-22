cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})

#Uncomment this for the tutorial
# add_executable (extract_indices src/tutorials/extract_indices.cpp)
# target_link_libraries (extract_indices ${PCL_LIBRARIES})

# add_executable (clustering src/tutorials/clustering.cpp)
# target_link_libraries (clustering ${PCL_LIBRARIES})

#For quiz
#add_subdirectory(src/quiz/ransac)
#add_subdirectory(src/quiz/cluster)




