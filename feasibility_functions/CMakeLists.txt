
add_library(feasibility_functions STATIC
  src/CheckAccelerationConstraint.cpp
  src/CheckCurvatureConstraints.cpp
  src/CheckCurvatureRateConstrains.cpp
  src/CheckVelocityConstraints.cpp
  src/CheckYawRateConstraint.cpp
)

# find_package(Eigen3 REQUIRED NO_MODULE)

target_include_directories(feasibility_functions PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # ${CMAKE_SOURCE_DIR}/eigen-3.4.0
)

# target_include_directories(feasibility_functions PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_libraries(feasibility_functions PUBLIC Eigen3::Eigen trajectory_sample)
