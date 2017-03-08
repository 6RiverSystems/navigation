/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/follower_trajectory_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

std::shared_ptr<base_local_planner::LocalPlannerLimits> getLimits()
{
  return std::make_shared<base_local_planner::LocalPlannerLimits>(
    1.0, // max_trans_vel
    0.1, // min_trans_vel
    1.0, // max_x_vel
    0.1, // min_x_vel
    0.0, // max_y_vel
    0.0, // min_y_vel
    2.0, // max_rot_vel
    0.1, // min_rot_vel
    1.0, // acc_lim_x
    0.0, // acc_lim_y
    2.0, // acc_lim_theta
    1.0, // acc_lim_trans
    0.15, // xy_gal_tolerance
    0.1  // yaw_goal_tolerance
    );
}

FollowerTrajectoryGenerator createFTG(base_local_planner::LocalPlannerLimits* lim)
{
  FollowerTrajectoryGenerator ftg = FollowerTrajectoryGenerator();
  ftg.setParameters(
    2.0, // sim_time
    0.1, // sim_granularity
    10.0, // kp_theta
    0.5, // lookahead_distance
    2 // num_trajectories
    );
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
  ftg.initialise(pos, vel, lim);
  return ftg;
}

Eigen::Vector3f createVector(float x, float y, float yaw)
{
  Eigen::Vector3f out = Eigen::Vector3f::Zero();
  out[0] = x;
  out[1] = y;
  out[2] = yaw;
  return out;
}

std::vector<geometry_msgs::PoseStamped> createSimpleGlobalPlan()
{
  // Create a global plan
  std::vector<geometry_msgs::PoseStamped> out;

  // Create a straight line down the x axis
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  for (x = 0.0; x <= 5.0; x += 0.1)
  {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    out.push_back(p);
  }
  return out;
}

void printTrajectory(const base_local_planner::Trajectory& traj)
{
  std::stringstream ss;

  ss << "Print trajectory" << std::endl
    << " xv: " << traj.xv_ << ", thetav: " << traj.thetav_ << std::endl
    << " cost: " << traj.cost_ << ", td: " << traj.time_delta_ << ", samples: " << traj.getPointsSize() << std::endl;

  for (unsigned int k = 0; k < traj.getPointsSize(); ++k)
  {
    double x, y, th;
    traj.getPoint(k, x, y, th);
    double vx, vy, vth;
    traj.getVelocity(k, vx, vy, vth);
    ss << "  " << k << ": [" << x << ", " << y << ", " << th << "] ["
      << vx << ", " << vy << ", " << vth << "]" << std::endl;
  }
  std::cerr << ss.str();
}

TEST(FollowerTrajectoryGenerator, basic){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());
  EXPECT_EQ(0, 0);
}

TEST(FollowerTrajectoryGenerator, follow_on_test){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());

  // Add a global plan
  ftg.setGlobalPlan(createSimpleGlobalPlan());

  // Ask for a trajectory.
  base_local_planner::Trajectory traj;

  bool res = ftg.generateTrajectory(
    createVector(0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    0.7, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);

  res = ftg.generateTrajectory(
    createVector(0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    1.7, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);
}

TEST(FollowerTrajectoryGenerator, near_end_test){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());

  // Add a global plan
  ftg.setGlobalPlan(createSimpleGlobalPlan());

  // Ask for a trajectory.
  base_local_planner::Trajectory traj;

  bool res = ftg.generateTrajectory(
    createVector(0.0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    5.0, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);
}


}
