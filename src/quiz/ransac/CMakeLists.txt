cmake_minimum_required(VERSION 3.10)
project(quizRansac)

# Set C++17 standard (required for PCL 1.15 and std::filesystem)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find PCL
find_package(PCL REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
# Add your render directory to include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../render)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(quizRansac 
    ${CMAKE_CURRENT_SOURCE_DIR}/ransac2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../render/render.cpp
)

# Link libraries including filesystem
target_link_libraries(quizRansac ${PCL_LIBRARIES})

# For older compilers that need explicit filesystem linking
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
    target_link_libraries(quizRansac stdc++fs)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
    target_link_libraries(quizRansac c++fs)
endif()
