cmake_minimum_required(VERSION 2.8.3)
project(base_local_planner)
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

include(CheckIncludeFile)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            geometry_msgs
            message_generation
            nav_core
            nav_msgs
            pluginlib
<<<<<<< HEAD
            angles
            srslib_timing
=======
            roscpp
            rospy
            sensor_msgs
            std_msgs
            tf2
            tf2_geometry_msgs
            tf2_ros
            voxel_grid
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
        )

find_package(Eigen3 REQUIRED)
<<<<<<< HEAD
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io)

=======
remove_definitions(-DDISABLE_LIBUSB-1.0)
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_python_setup()

# messages
add_message_files(
    DIRECTORY msg
    FILES
        Position2DInt.msg
        CriticCosts.msg
        ScoredSamplingPlannerCosts.msg
        SpeedLimitRequest.msg
        Obstacle.msg
        Obstacles.msg
        SpeedLimiterMsg.msg
        SpeedLimitersMsg.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/BaseLocalPlanner.cfg
    cfg/SpeedLimitManager.cfg
    cfg/ExternalSpeedLimiter.cfg
    cfg/ObstacleSpeedLimiter.cfg
    cfg/PathSpeedLimiter.cfg
    cfg/ShadowSpeedLimiter.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        base_local_planner
        trajectory_planner_ros
    CATKIN_DEPENDS
        angles
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_runtime
        nav_core
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        voxel_grid
)

check_include_file(sys/time.h HAVE_SYS_TIME_H)
if (HAVE_SYS_TIME_H)
  add_definitions(-DHAVE_SYS_TIME_H)
endif (HAVE_SYS_TIME_H)

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

add_library(base_local_planner
<<<<<<< HEAD
    src/critics/euclidean_distance_cost_function.cpp
    src/critics/velocity_cost_function.cpp
    src/critics/heading_cost_function.cpp
    src/critics/global_plan_distance_cost_function.cpp
    src/critics/jerk_cost_function.cpp
    src/critics/map_grid_cost_function.cpp
    src/critics/obstacle_cost_function.cpp
    src/critics/oscillation_cost_function.cpp
    src/critics/prefer_forward_cost_function.cpp
    src/follower_trajectory_generator.cpp
    src/footprint_helper.cpp
    src/geometry_math_helpers.cpp
    src/goal_functions.cpp
    src/map_cell.cpp
    src/map_grid.cpp
    src/map_grid_visualizer.cpp
    src/latched_stop_rotate_controller.cpp
    src/local_planner_util.cpp
    src/odometry_helper_ros.cpp
    src/point_and_shoot_trajectory_generator.cpp
    src/point_grid.cpp
    src/costmap_model.cpp
    src/simple_scored_sampling_planner.cpp
    src/simple_trajectory_generator.cpp
    src/speed_limiters/speed_limit_manager.cpp
    src/speed_limiters/shadow_speed_limiter.cpp
    src/speed_limiters/path_speed_limiter.cpp
    src/speed_limiters/obstacle_speed_limiter.cpp
    src/speed_limiters/external_speed_limiter.cpp
    src/stationary_trajectory_generator.cpp
    src/turn_in_place_trajectory_generator.cpp
    src/trajectory.cpp
    src/voxel_grid_model.cpp)

add_dependencies(base_local_planner
    base_local_planner_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})
=======
	src/footprint_helper.cpp
	src/goal_functions.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/map_grid_cost_function.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/point_grid.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
	src/twirling_cost_function.cpp
	src/voxel_grid_model.cpp)
add_dependencies(base_local_planner base_local_planner_gencfg)
add_dependencies(base_local_planner base_local_planner_generate_messages_cpp)
add_dependencies(base_local_planner nav_msgs_generate_messages_cpp)
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
target_link_libraries(base_local_planner
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_library(trajectory_planner_ros
<<<<<<< HEAD
    src/trajectory_planner.cpp
    src/trajectory_planner_ros.cpp)
add_dependencies(trajectory_planner_ros
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner_ros
     base_local_planner)

add_executable(point_grid src/point_grid.cpp)
target_link_libraries(point_grid
    ${catkin_LIBRARIES})
=======
	src/trajectory_planner.cpp
	src/trajectory_planner_ros.cpp)
add_dependencies(trajectory_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner_ros
     base_local_planner)

add_executable(point_grid src/point_grid_node.cpp)
add_dependencies(point_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_grid ${catkin_LIBRARIES} base_local_planner)
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691

install(TARGETS
            base_local_planner
            trajectory_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  catkin_add_gtest(base_local_planner_utest
    test/gtest_main.cpp
    test/utest.cpp
    test/velocity_iterator_test.cpp
    test/footprint_helper_test.cpp
    test/geometry_math_helpers_test.cpp
    test/local_planner_limits_test.cpp
    test/trajectory_generator_test.cpp
    test/follower_trajectory_generator_test.cpp
    test/global_plan_distance_cost_function_test.cpp
    test/map_grid_test.cpp
    test/critic_test_helpers.cpp
    test/odometry_helper_ros_test.cpp
    )
  target_link_libraries(base_local_planner_utest
      base_local_planner trajectory_planner_ros
      )

  catkin_add_gtest(line_iterator
      test/line_iterator_test.cpp)
endif()
