/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: dgrieneisen
 *********************************************************************/

#ifndef FOLLOWER_TRAJECTORY_GENERATOR_H_
#define FOLLOWER_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner {

/**
 * generates trajectories based a simple path following controller.
 */
class FollowerTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:

  FollowerTrajectoryGenerator() {
    limits_ = NULL;
  }

  ~FollowerTrajectoryGenerator() {}

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      base_local_planner::LocalPlannerLimits* limits);

  /**
   * This function is to be called only when parameters change
   *
   * @param sim_time length of time for the simulation
   * @param sim_granularity granularity of collision detection
   * @param kp_theta proportional controller constant on heading
   * @param lookahead_distance distance ahead on the path to look for heading goal
   * @param num_trajectories the number of different trajectories to create
   */
  void setParameters(double sim_time,
      double sim_granularity,
      double kp_theta,
      double lookahead_distance,
      double num_trajectories);

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(Trajectory &traj);

  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

  Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& position,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt,
      unsigned int start_idx, unsigned int& closest_idx);

  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        double max_sim_time,
        base_local_planner::Trajectory& traj);

  virtual double getStartLinearVelocity() { return vel_[0];};
  virtual double getStartAngularVelocity() { return vel_[2];};

  void setSimTime(double sim_time)
  {
    sim_time_ = sim_time;
  }

  void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    global_plan_ = global_plan;
  }

protected:

  void getDesiredHeadingAndGoalDistance(const Eigen::Vector3f& pos,
    unsigned int start_idx, unsigned int& closest_idx,
    double& desired_heading, double& distance_to_goal);

  unsigned int next_sample_index_;
  // to store sample params of each sample between init and generation
  std::vector<double> end_times_;
  base_local_planner::LocalPlannerLimits* limits_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;

  double sim_time_;
  double sim_granularity_;

  double kp_theta_;
  double lookahead_distance_;
  unsigned int num_trajectories_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  base_local_planner::Trajectory stored_trajectory_;
  double stored_trajectory_end_time_;
  unsigned int stored_idx_;
};

} /* namespace base_local_planner */
#endif /* FOLLOWER_TRAJECTORY_GENERATOR_H_ */
