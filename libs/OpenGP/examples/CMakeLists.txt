if(NOT BUILD_EXAMPLES)
    return()
else()
    message(STATUS "==> ${CMAKE_CURRENT_LIST_DIR}")
endif()

#--- OpenGP (headeronly)
find_package(OpenGP REQUIRED)
include_directories(${OpenGP_INCLUDE_DIR})

#--- Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(-DUSE_EIGEN)

#--- Compile examples one by one
add_executable(surface_mesh_basics      surface_mesh_basics.cpp)
add_executable(surface_mesh_iterators   surface_mesh_iterators.cpp)
add_executable(surface_mesh_io          surface_mesh_io.cpp)
add_executable(surface_mesh_barycenter  surface_mesh_barycenter.cpp)
add_executable(surface_mesh_c++11       surface_mesh_c++11.cpp)
add_executable(global_properties        global_properties.cpp)
