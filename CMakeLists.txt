cmake_minimum_required(VERSION 3.12.4) #command add_compile_definitions is added in 3.12.4
project(moveit_no_ros)
include(GNUInstallDirs)

## Compile as C++11
add_compile_options(-std=c++11)

OPTION(WITH_BULLET3 "Whether build library or executable program depend on bullet3" OFF)
IF(${WITH_BULLET3})
    MESSAGE(STATUS "Build with bullet3")
    find_package(Bullet REQUIRED)
    MESSAGE(STATUS "BULLET_INCLUDE_DIR ${BULLET_INCLUDE_DIR}")
    MESSAGE(STATUS "BULLET_LIBRARIES ${BULLET_LIBRARIES}")
    ADD_DEFINITIONS(-DWITH_BULLET3)
    find_library(bullet_robotics_lib BulletRobotics REQUIRED)
    MESSAGE(STATUS "lib BulletRobotics ${bullet_robotics_lib}")
ELSE()
    MESSAGE(STATUS "Do not build with bullet3")
ENDIF()

## Build in debug mode
SET(CMAKE_BUILD_TYPE "Debug")

## Set exported head file directory
SET(EXPORT_HEAD_DIR "exported_heads")

## Let tinyxml use stl string instead of tinystr
add_compile_definitions(TIXML_USE_STL)

SET(KINETIC_LIB_DIR ${PROJECT_SOURCE_DIR}/extern/lib)

SET(KINETIC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/extern/include)

## Set Python Version for pybind11
SET(PYBIND11_PYTHON_VERSION "3.8")

## Set yaml-cpp shared-library on
SET(BUILD_SHARED_LIBS "ON")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/install")

## Show install paths
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_INSTALL_BINDIR: ${CMAKE_INSTALL_BINDIR}")
message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")

SET(YAML_CPP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/extern/yaml-cpp/include)
SET(YAML_CPP_SRC_DIR ${PROJECT_SOURCE_DIR}/extern/yaml-cpp/src)

MESSAGE(STATUS "YAML_CPP_INCLUDE_DIR ${YAML_CPP_INCLUDE_DIR}")
MESSAGE(STATUS "YAML_CPP_SRC_DIR ${YAML_CPP_SRC_DIR}")

## pybind11是用来进行C++和python互相调用的库
find_package(Python ${PYBIND11_PYTHON_VERSION} REQUIRED COMPONENTS Interpreter Development)
MESSAGE(STATUS "Python_SITELIB: ${Python_SITELIB}")

### thread C++多线程包
find_package(Threads REQUIRED)

### Eigen3 C++矩阵库
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
MESSAGE(STATUS "include Eigen3: ${EIGEN3_INCLUDE_DIRS}")

## boost::filesystem是Boost C++ Libraries中的一个模块，
find_package(Boost COMPONENTS system filesystem REQUIRED)

## 科学数据包
find_library(lapack_lib lapack
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lapack: ${lapack_lib}")

find_library(blas_lib blas
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "blas: ${blas_lib}")

find_library(gfortran_lib gfortran
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "gfortran: ${gfortran_lib}")

# moveit 相关的包文件 
## 变量名称 + 库名称  rdf_loader_lib moveit_rdf_loader
## 1.rdf_loader_lib //moveit_rdf_loader
find_library(rdf_loader_lib moveit_rdf_loader
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_rdf_loader: ${rdf_loader_lib}")

## 2.robot_model_lib //moveit_robot_model
find_library(robot_model_lib moveit_robot_model
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_model: ${robot_model_lib}")

# 3.kinematics_base_lib // moveit_kinematics_base
find_library(kinematics_base_lib moveit_kinematics_base
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_kinematics_base: ${kinematics_base_lib}")
# 4.orocos_kdl_lib // orocos-kdl
find_library(orocos_kdl_lib orocos-kdl # can also installed as third party library: https://orocos.org/kdl.html
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib orocos-kdl: ${orocos_kdl_lib}")

# 5.robot_state_lib // moveit_robot_state
find_library(robot_state_lib moveit_robot_state
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_state: ${robot_state_lib}")

# 6.kdl_parser_lib  // kdl_parser
find_library(kdl_parser_lib kdl_parser
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib kdl_parser: ${kdl_parser_lib}")

# 7.kdl_conversions_lib kdl_conversions
find_library(kdl_conversions_lib kdl_conversions
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib kdl_conversions: ${kdl_conversions_lib}")

# 8.collision_detection_lib // moveit_collision_detection
find_library(collision_detection_lib moveit_collision_detection
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_collision_detection: ${collision_detection_lib}")

# 9.planning_scene_lib moveit_planning_scene
find_library(planning_scene_lib moveit_planning_scene
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_planning_scene: ${planning_scene_lib}")

# 10.planning_interface_lib moveit_planning_interface
find_library(planning_interface_lib moveit_planning_interface
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_planning_interface: ${planning_interface_lib}")

# 11.ompl_interface_lib moveit_ompl_interface
find_library(ompl_interface_lib moveit_ompl_interface
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_ompl_interface: ${ompl_interface_lib}")

# 12.constraint_samplers_lib moveit_constraint_samplers
find_library(constraint_samplers_lib moveit_constraint_samplers
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_constraint_samplers: ${constraint_samplers_lib}")

# 13.robot_trajectory_lib moveit_robot_trajectory
find_library(robot_trajectory_lib moveit_robot_trajectory
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_robot_trajectory: ${robot_trajectory_lib}")

# 14.trajectory_processing_lib moveit_trajectory_processing
find_library(trajectory_processing_lib moveit_trajectory_processing
        PATHS ${KINETIC_LIB_DIR}
        NO_DEFAULT_PATH)
MESSAGE(STATUS "lib moveit_trajectory_processing: ${trajectory_processing_lib}")


function(path_indent arg_path)
    execute_process(COMMAND sh path_indent.sh "${arg_path}" ";" "......" OUTPUT_VARIABLE shell_out WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    if (NOT ${shell_out} EQUAL "") # It may raise an error if argument of message is empty.
        message(${shell_out})
    endif (NOT ${shell_out} EQUAL "")
endfunction()

add_subdirectory(extern/pybind11)
add_subdirectory(src/yaml-cpp)
add_subdirectory(extern/kdl_kinematics_plugin)
add_subdirectory(src/robot_model)
add_subdirectory(src/planner)
add_subdirectory(src/controller)
add_subdirectory(src/python)

include_directories(
    ${KINETIC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    src/robot_model/include
    src/planner/include
    src/rokae/include
    extern
    ${PROJECT_SOURCE_DIR}/extern/include
)

add_executable(urdf_converter src/util/urdf_converter.cpp)


install(TARGETS  urdf_converter
        ARCHIVE DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY resources
        DESTINATION ${CMAKE_INSTALL_BINDIR})
