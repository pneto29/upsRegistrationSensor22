cmake_minimum_required(VERSION 3.10)
project(UPS_Point_Cloud_Registration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS common io registration visualization search kdtree)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Some older PCL installations expose an unavailable vtkproj4 library.
list(REMOVE_ITEM PCL_LIBRARIES vtkproj4)

add_executable(ups_config_a src/ups_config_a.cpp)
add_executable(ups_config_b src/ups_config_b.cpp)
target_link_libraries(ups_config_a ${PCL_LIBRARIES})
target_link_libraries(ups_config_b ${PCL_LIBRARIES})
