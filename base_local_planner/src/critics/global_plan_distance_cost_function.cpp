/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, 6 River Systems
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
 * Author: Daniel Grieneisen
 *********************************************************************/

#include <base_local_planner/critics/global_plan_distance_cost_function.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

GlobalPlanDistanceCostFunction::GlobalPlanDistanceCostFunction(double max_distance_from_plan) :
    max_allowed_distance_from_plan_(max_distance_from_plan),
    distance_violation_(false) {}

void GlobalPlanDistanceCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool GlobalPlanDistanceCostFunction::prepare() {

  distance_violation_ = true;

  if (target_poses_.size() == 0)
  {
    distance_violation_ = false;
    return true;
  }

  // Go forwards through the poses and see if any are within the minimum distance
  double distance_limit_squared = max_allowed_distance_from_plan_ * max_allowed_distance_from_plan_;

  for (size_t k = 0; k < target_poses_.size(); ++k)
  {
    if (poseDistanceSquared(target_poses_[k], current_pose_) < distance_limit_squared)
    {
      distance_violation_ = false;
      break;
    }
  }
  if (distance_violation_)
  {
    ROS_WARN("Global plan is too far from the current robot pose.");
  }
  return true;
}

double GlobalPlanDistanceCostFunction::scoreTrajectory(Trajectory &traj) {

  if (distance_violation_)
  {
    // Check to see if the trajectory is coming to a stop
    if (std::fabs(traj.thetav_) > EPSILON || std::fabs(traj.xv_) > EPSILON)
    {
      return -1.0;
    }
  }
  return 0.0;
}

} /* namespace base_local_planner */
