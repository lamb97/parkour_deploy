# 在CMakeLists.txt的开  头，定义TIMEPRINT宏并设置其值为true
add_definitions(-DTIMEPRINT=TRUE)

cmake_minimum_required(VERSION 3.10)
project(image_mnn)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries     
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        control_msg
        std_msgs
        message_generation
        rospy
        tf2
        tf
        cv_bridge
        image_transport
        )


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        control_msg
        message_generation
        message_runtime
        cv_bridge
        image_transport
        tf
        tf2
        DEPENDS
        MNN
        OpenCV
)

###########
## Build ##
###########
# 添加头文件搜索路径
include_directories(/usr/local/include
                    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(image_mnn_node src/main.cpp src/ImageNode.cpp)

# 添加链接库
target_link_libraries(image_mnn_node
        ${catkin_LIBRARIES}
        MNN
        ${OpenCV_LIBS}
        )

## Declare a cpp executable

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR   }/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS image_mnn_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
# install(FILES legged_controllers_plugins.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         )
