# Updated CMakeLists.txt to include render source files

cmake_minimum_required(VERSION 3.10)
project(quizCluster)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find PCL
find_package(PCL REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
# Add your render directory to include path
include_directories(../render)  # Path to src/render from src/quiz/cluster

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Include render.cpp in your executable
add_executable(quizCluster 
    cluster.cpp
    /Users/moorissa/Repos/sensor_fusion/lidar-obstacle-detector/src/render/render.cpp  # Correct path: src/render/render.cpp
)

# SOLUTION 2: Alternative - Create a render library first
# add_library(render_lib ../render/render.cpp)
# add_executable(quizCluster cluster.cpp)
# target_link_libraries(quizCluster render_lib ${PCL_LIBRARIES})

# Link libraries
target_link_libraries(quizCluster ${PCL_LIBRARIES})

# If you need additional PCL components, specify them:
# find_package(PCL REQUIRED COMPONENTS common io visualization)
# target_link_libraries(quizCluster ${PCL_LIBRARIES})