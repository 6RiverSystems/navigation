/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/global_plan_distance_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

constexpr float DIST = 0.5;

GlobalPlanDistanceCostFunction createGPDCF()
{
  GlobalPlanDistanceCostFunction gpdcf = GlobalPlanDistanceCostFunction(DIST);
  return gpdcf;
}

geometry_msgs::PoseStamped createPoseStamped(float x, float y, float yaw)
{
  geometry_msgs::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}

std::vector<geometry_msgs::PoseStamped> createGlobalPlan()
{
  // Create a global plan
  std::vector<geometry_msgs::PoseStamped> out;

  // Create a straight line down the x axis
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  for (x = 0.0; x <= 5.0; x += 0.1)
  {
    out.push_back(createPoseStamped(x, y, yaw));
  }
  return out;
}

Trajectory createTrajectory(double v, double w)
{
  return Trajectory(v, 0, w, 0.1, 1);
}


TEST(GlobalPlanDistanceCostFunction, test)
{

  GlobalPlanDistanceCostFunction cf = createGPDCF();
  Trajectory traj;

  // With no plan or pose, it should return true.
  cf.prepare();
  traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  // Set a trajectory
  cf.setTargetPoses(createGlobalPlan());
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  cf.prepare();
  traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  // Now test with a pose far away.
  cf.setCurrentPose(createPoseStamped(2 * DIST, 0, 0));
  cf.prepare();
  // Velocity of 0 is still valid
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  // Velocity > 0 is not valid
  traj = createTrajectory(1, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  traj = createTrajectory(0, 2);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  traj = createTrajectory(1, 1);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

}
