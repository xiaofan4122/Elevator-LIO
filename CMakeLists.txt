cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

project(lio LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)

set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "Disable catkin tests" FORCE)

set(CMAKE_CXX_FLAGS "-O3 -march=native ${CMAKE_CXX_FLAGS}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


# 确保在安装时不清除 RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 强制将系统库目录加入优先搜索列表
SET(CMAKE_BUILD_RPATH "/usr/lib/x86_64-linux-gnu")


set(CURRENT_USER $ENV{USER})
message(STATUS "当前用户是: ${CURRENT_USER}")

# 开启OpenMP（多线程）
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        pcl_ros
        pcl_conversions
        tf2_ros
        message_generation
        std_srvs
        rviz
        pluginlib
)
# 找到OpenCV库
find_package(OpenCV REQUIRED)
# 包含OpenCV的头文件
include_directories(${OpenCV_INCLUDE_DIRS})
set(YAML_CPP_INCLUDE_DIR /usr/include/yaml-cpp)
set(YAML_CPP_LIBRARY /usr/lib/x86_64-linux-gnu/libyaml-cpp.so)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_message_files(
        DIRECTORY msg
        FILES
        CustomMsg.msg   # livox_ros_driver2 CustomMsg
        CustomPoint.msg # livox_ros_driver2 CustomPoint
        wheel_info.msg  # yhs wheel info
        ElevatorState.msg
)

# 包含头文件目录
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        include
        include/ikd_tree
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lio_rviz_plugins
        DEPENDS EIGEN3 PCL
        CATKIN_DEPENDS
        std_msgs
        nav_msgs
        geometry_msgs
        message_runtime
        rviz
        pluginlib
)

qt5_wrap_cpp(ELEVATOR_STATUS_PANEL_MOC
        include/rviz/ElevatorStatusPanel.h
)

add_library(lio_rviz_plugins
        src/rviz/ElevatorStatusPanel.cpp
        ${ELEVATOR_STATUS_PANEL_MOC}
)

add_dependencies(lio_rviz_plugins
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lio_rviz_plugins
        ${catkin_LIBRARIES}
        Qt5::Widgets
)

# 添加可执行文件
add_executable(lio
        include/support/YamlReader.cpp
        include/support/YamlReader.h
        src/main.cpp
        include/support/LIONode.h
        src/support/TopicReceive.cpp
        include/support/TopicProcess.h
        src/support/common_lib.cpp
        include/support/common_lib.h
        src/estimator/ESEKF.cpp
        include/estimator/ESEKF.h
        include/ikd_tree/IkdNearestQuery.hpp
        include/ikd_tree/IkdMap.hpp
        include/ikd_tree/ikd_Tree.h
        include/ikd_tree/ikd_Tree.cpp
        include/support/SharedBuffers.h
        include/support/type.h
        src/node/publish.cpp
        src/node/callbacks.cpp
        src/node/map.cpp
        src/node/save.cpp
        src/node/LidarPipeline.cpp
        src/estimator/IMUProcess.cpp
        include/estimator/IMUProcess.h
        src/elevator/ElevatorProcess.cpp
        src/elevator/ElevatorSelfExit.cpp
        include/elevator/ElevatorProcess.h
        include/elevator/ElevatorSelfExit.h
        src/node/service.cpp
        include/AdaptiveFilter/AdaptiveVoxelFilter.hpp
        include/AdaptiveFilter/AdaptiveVoxelPController.hpp
)

add_dependencies(lio
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(lio
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        yaml-cpp
        ${OpenCV_LIBS}
        tbb
)
# 定义宏
target_compile_definitions(lio PRIVATE PACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


# 安装可执行文件
install(TARGETS lio
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(sim_node
        src/sim/run_sim_node.cpp
        src/sim/ElevatorTrajectory.hpp
        src/sim/ObjectRelativeTrajectory.hpp
        src/sim/ObjectWorldTrajectory.hpp
        src/sim/SimSensorPublisher.hpp
)

target_link_libraries(sim_node
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
)

target_compile_definitions(sim_node PRIVATE PACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 安装可执行文件
install(TARGETS lio_rviz_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS sim_node
        DESTINATION lib/${PROJECT_NAME}
)
