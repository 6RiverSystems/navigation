cmake_minimum_required(VERSION 2.8.3)
project(fake_localization)
add_definitions("-std=c++11")

find_package(catkin REQUIRED
        COMPONENTS
            angles
            geometry_msgs
            message_filters
            nav_msgs
            rosconsole
            roscpp
            rospy
            tf2_geometry_msgs
            tf2_ros
        )


find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fake_localization fake_localization.cpp)
target_link_libraries(fake_localization
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
<<<<<<< HEAD

add_dependencies(fake_localization ${catkin_EXPORTED_TARGETS})
=======
add_dependencies(fake_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691

install(
    PROGRAMS
       static_odom_broadcaster.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        fake_localization
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
