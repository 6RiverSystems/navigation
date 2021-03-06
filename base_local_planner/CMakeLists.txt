cmake_minimum_required(VERSION 2.8.3)
project(base_local_planner)
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            message_generation
            dynamic_reconfigure
            nav_core
            pcl_conversions
            rostest
            costmap_2d
            pluginlib
            angles
            srslib_framework
            srslib_timing
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

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
        StaticObjects.msg
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
    cfg/StaticObjectSpeedLimiter.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        base_local_planner
        trajectory_planner_ros
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        message_generation
        tf
        pluginlib
        costmap_2d
        nav_core
        angles
        srslib_timing
        srslib_framework
)

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

add_library(base_local_planner
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
    src/speed_limiters/static_object_speed_limiter.cpp
    src/stationary_trajectory_generator.cpp
    src/turn_in_place_trajectory_generator.cpp
    src/trajectory.cpp
    src/voxel_grid_model.cpp)

add_dependencies(base_local_planner
    base_local_planner_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_local_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    )

add_library(trajectory_planner_ros
    src/trajectory_planner.cpp
    src/trajectory_planner_ros.cpp)
add_dependencies(trajectory_planner_ros
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner_ros
     base_local_planner)

add_executable(point_grid src/point_grid.cpp)
target_link_libraries(point_grid
    ${catkin_LIBRARIES})

install(TARGETS
            base_local_planner
            trajectory_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
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
    test/path_speed_limiter_test.cpp
    test/static_object_speed_limiter_test.cpp
    )

  target_link_libraries(base_local_planner_utest
      base_local_planner trajectory_planner_ros
      )


  catkin_add_gtest(line_iterator
      test/line_iterator_test.cpp)
endif()
