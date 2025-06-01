cmake_minimum_required(VERSION 3.12)
project(MyProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Treat Eigen as an interface library (recommended)
add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE 
    ${CMAKE_SOURCE_DIR}/eigen-3.4.0
)

add_subdirectory(cost_functions)
add_subdirectory(feasibility_functions)
add_subdirectory(trajectory_sample)
add_subdirectory(trajectory_handler)
add_subdirectory(third_party/crdc)

# Add sources
# file(GLOB SOURCES "src/*.cpp" "main.cpp")
add_executable(myapp main.cpp)

# Link dependencies
# target_include_directories(myapp PRIVATE include/)
target_link_libraries(myapp PRIVATE 
    Eigen
    cost_functions
    feasibility_functions
    trajectory_sample
    trajectory_handler
)