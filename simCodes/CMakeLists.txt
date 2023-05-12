cmake_minimum_required(VERSION 3.10)
project(HelixSimulation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../")


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(
        ${EIGEN_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        )


add_executable(simDER
        src/main.cpp
        src/world.cpp
        src/elasticRod.cpp
        src/elasticStretchingForce.cpp
        src/elasticBendingForce.cpp
        src/elasticTwistingForce.cpp
        src/externalGravityForce.cpp
        src/dampingForce.cpp
        src/inertialForce.cpp
        src/timeStepper.cpp
        src/setInput.cpp
        )


target_link_libraries(simDER
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        Eigen3::Eigen
        lapack
        gfortran
        pthread
        rt
        m)
