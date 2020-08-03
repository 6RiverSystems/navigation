cmake_minimum_required(VERSION 2.8.3)
project(move_slow_and_clear)
add_definitions("-std=c++11")

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        nav_core
        pluginlib
        roscpp
        )


find_package(Eigen3 REQUIRED)
<<<<<<< HEAD
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io)
=======
remove_definitions(-DDISABLE_LIBUSB-1.0)
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES move_slow_and_clear
    CATKIN_DEPENDS
        geometry_msgs
        nav_core
        pluginlib
        roscpp
)

add_library(${PROJECT_NAME} src/move_slow_and_clear.cpp)
<<<<<<< HEAD
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
=======
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)

install(TARGETS move_slow_and_clear
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


