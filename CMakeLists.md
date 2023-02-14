# CmakeLists 说明
### Cmake最低版本要求3.14.2
## cmake_minimum_required(VERSION 3.12.4) 
---------------------------
### 项目名称moveit_no_ros
## project(moveit_no_ros)  
---------------------------
### 使用 GNUInstallDirs.cmake 定义目标安装的标准位置。
## include(GNUInstallDirs) 
---------------------------
<!-- 使用 GNUInstallDirs.cmake 定义目标安装的标准位置。
    GNUInstallDirs.cmake模块定义了一组变量，这些变量是安装不同类型文件的子目录的名称，有助于将已安装的文件放置到所选安装前缀的子目录中。

CMAKE_INSTALL_BINDIR：这将用于定义用户可执行文件所在的子目录，即所选安装目录下的 bin 目录。
CMAKE_INSTALL_LIBDIR：这将扩展到目标代码库(即静态库和动态库)所在的子目录。在64位、           系统上，它是 lib64 ，而在32位系统上，它只是 lib 。
CMAKE_INSTALL_INCLUDEDIR：使用这个变量为头文件获取正确的子目录，该变量为 include。


include(GNUInstallDirs)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}) -->
### 想在编译C++代码支持C++11的选项
## add_compile_options(-std=c++11)
<!-- add_compile_options添加编译选项（-g -Werror） -->
---------------------------

### CMake中的option用于控制编译流程，相当于C语言中的宏条件编译。构建库还是可执行程序取决于 bullet3
## OPTION(WITH_BULLET3 "Whether build library or executable program depend on bullet3" OFF) 
    OPTION(WITH_BULLET3 "Whether build library or executable program depend on bullet3" OFF)
    #IF(${WITH_BULLET3})
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
---------------------------
<!-- variable：定义选项名称
help_text：说明选项的含义
value:定义选项默认状态，一般是OFF或者ON，除去ON之外，其他所有值都为认为是OFF。 -->
<!-- 
美元符号 $，扩展打开 makefile 中定义的变量。

MESSAGE(STATUS "Build with bullet3")  打印开始日志

find_package(Bullet REQUIRED)
CMake给我们提供了find_package()命令用来查找依赖包，理想情况下，

一句find_package()命令就能把一整个依赖包的头文件包含路径、库路径、库名字、版本号等情况都获取到，后续只管用就好了

REQUIRED可选字段。表示一定要找到包，找不到的话就立即停掉整个cmake。而如果不指定REQUIRED则cmake会继续执行

option的名字保持和main.c里的宏名称一致，这样更加直观，也可以选择不同的名字。

add_definitions的功能和C/C++中的#define是一样的add_definitions的功能和C/C++中的#defin
 --> 
 ### set方法是cmake-commands中的脚本方法，用于给下面的变量设置值：  说明版本为Debug
<!-- 一般变量(Set Normal Variable)
缓存变量(Set Cache Entry)
环境变量(Set Environment Variable) -->
## SET(CMAKE_BUILD_TYPE "Debug")
---------------------------

### 设置导出的头文件目录
## SET(EXPORT_HEAD_DIR "exported_heads")
---------------------------

### 让 tinyxml 使用 stl 字符串而不是 tinystr
### 将预处理器定义添加到源文件的编译中
## add_compile_definitions(TIXML_USE_STL)
---------------------------

### thread python多线程包
## find_package(Threads REQUIRED)
---------------------------

### Eigen3 C++矩阵库
## find_package(Eigen3 REQUIRED)
---------------------------

### boost::filesystem是Boost C++ Libraries中的一个模块，主要作用是处理文件（Files）和目录(Directories)。
### 该模块提供的类boost::filesystem::path专门用来处理路径。而且，该模块中还有很多独立的函数能够用来执行创建目录、检查文件是否存在等任务。
## find_package(Boost COMPONENTS system filesystem REQUIRED)
---------------------------

# moveit相关包文件
#### robot_description 加载 URDF 的 ROS 参数对应的字符串名称
    find_library(rdf_loader_lib moveit_rdf_loader
            PATHS ${KINETIC_LIB_DIR}
            NO_DEFAULT_PATH
#### https://docs.ros.org/en/lunar/api/moveit_ros_planning/html/rdf__loader_8h_source.html
### 1.find_library(rdf_loader_lib moveit_rdf_loader
<!-- find_library: 在指定目录下查找指定库，并把库的绝对路径存放到变量里，其第一个参数是变量名称，第二个参数是库名称，第三个参数是HINTS，第4个参数是路径，其它用法可以参考cmake文档
target_link_libraries: 把目标文件与库文件进行链接
使用find_library的好处是在执行cmake ..时就会去查找库是否存在，这样可以提前发现错误，不用等到链接时-->
---------------------------
### 机器人模型库  RobotModel类包含所有链接和关节之间的关系，包括从URDF加载的它们的关节限制属性。
### RobotModel还将机器人的链接和关节分离到SRDF中定义的规划组中。关于URDF和SRDF的单独教程可以在这里找到:URDF和SRDF教程
## 2.find_library(robot_model_lib moveit_robot_model
---------------------------
### KDL Orocos项目提供RealTime可用的运动学和动力学代码，它包含刚体运动学计算和表示的运动学结构及其逆和正运动学求解器的代码。
### https://github.com/orocos/orocos_kinematics_dynamics
## 3.find_library(orocos_kdl_lib orocos-kdl
---------------------------
### RobotState包含了机器人在某个时间点的信息，存储了关节位置的向量，以及可选的速度和加速度，
### 这些向量可用于获得机器人的运动学信息，这些信息取决于机器人的当前状态，例如末端执行器的雅可比矩阵。
## 4.find_library(robot_state_lib moveit_robot_state
---------------------------
### 运动学求解器 主要是运动学，逆运动学，包括了碰撞检测
## 5.find_library(kinematics_base_lib moveit_kinematics_base
---------------------------
###  KDL  从URDF文件创建KDL树
## 6.find_library(kdl_parser_lib kdl_parser
---------------------------
###  KDL KDL和geometry_msgs类型之间的转换函数
###  geometry_msgs主要是一些几何信息相关的数据结构
## https://blog.csdn.net/weixin_43699666/article/details/112344823
## 7.find_library(kdl_conversions_lib kdl_conversions
---------------------------
### 碰撞检测
### https://docs.ros.org/en/indigo/api/moveit_core/html/structcollision__detection_1_1CollisionRequest.html#details
## 8.find_library(collision_detection_lib moveit_collision_detection
---------------------------
### PlanningScene类提供了将用于collision检查和constraint检查的主接口
## 9.find_library(planning_scene_lib moveit_planning_scene
---------------------------
### moveit::planning_interface::MoveGroupInterface是moveit编程接口类，里面的成员函数几乎涵盖了所有控制机械臂的操作指令
## 10.find_library(planning_interface_lib moveit_planning_interface
---------------------------
### ompl 与 moveit 的接口
## 11.find_library(ompl_interface_lib moveit_ompl_interface
---------------------------
### 一些规划问题需要更复杂的或自定义的约束采样器来解决更困难的规划问题。
### 这个文档解释了如何创建一个自定义的运动规划约束采样器来使用MoveIt。
## 12.find_library(constraint_samplers_lib moveit_constraint_samplers
---------------------------
### 机器人轨迹库
## 13.find_library(robot_trajectory_lib moveit_robot_trajectory
---------------------------
### 轨迹处理库 此类修改轨迹的时间戳以尊重速度和加速度约束
## 14.find_library(trajectory_processing_lib moveit_trajectory_processing

-------------------------
# 以上为moveit相关的库文件


### 科学计算包
## find_library(lapack_lib lapack
---------------------------
### 科学计算包
## find_library(blas_lib blas
---------------------------
### gfortran 编译器
## find_library(gfortran_lib gfortran
---------------------------
###  pybind11 c++ pybind11是用来进行C++和python互相调用的库
## find_package(Python ${PYBIND11_PYTHON_VERSION} REQUIRED COMPONENTS Interpreter Development)
---------------------------
### 
## 
---------------------------
### 
## 