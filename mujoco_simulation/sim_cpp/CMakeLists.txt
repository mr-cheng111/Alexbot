cmake_minimum_required(VERSION 3.14)
project(alexbot_control_cpp)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PROJECT_SOURCE_DIR}/inc)

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(MUJOCO REQUIRED mujoco)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${MUJOCO_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    include
)

link_directories(
    ${MUJOCO_LIBRARY_DIRS}
    ${PINOCCHIO_LIBRARY_DIRS}
)

add_executable(robot_main src/main.cpp src/robot_sim.cpp src/robot_dynamics.cpp)

target_link_libraries(robot_main
    ${MUJOCO_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    dl
    glfw
)
