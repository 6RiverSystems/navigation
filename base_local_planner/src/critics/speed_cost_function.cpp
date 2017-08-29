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

#include <base_local_planner/critics/speed_cost_function.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

SpeedCostFunction::SpeedCostFunction() :
    max_linear_velocity_(1.0),
    min_linear_velocity_(0.0),
    max_allowed_vel_(1.0),
    linear_acceleration_(0.7),
    circumscribed_radius_(0.6),
    half_angle_(1.6),
    min_distance_to_stop_(0.0),
    max_distance_to_stop_(0.0),
    body_frame_id_("base_link"),
    world_frame_id_("odom") {}

bool SpeedCostFunction::prepare() {
  // Reset the maximum allowed velocity
  max_allowed_vel_ = max_linear_velocity_;
  if (!obstructions_)
  {
    ROS_WARN_THROTTLE(1.0, "No obstructions");
    return true;
  }

  // Find the nearest obstruction to the robot that is within the allowed range
  // Loop over all of the obstructions
  for (auto obs : (*obstructions_))
  {
    // Skip non-dynamic things or things that have been cleared
    if (obs.type != costmap_2d::ObstructionMsg::DYNAMIC || obs.cleared)
    {
      continue;
    }
    costmap_2d::ObstructionMsg obs_body_frame = obstructionToBodyFrame(obs);
    // Check if the bearing to the obstacle is acceptable
    if (std::fabs(getBearingToObstacle(obs_body_frame)) > half_angle_)
    {
      continue;
    }

    // Now calculate the speed
    double speed = calculateAllowedSpeed(obs_body_frame);
    if (speed < max_allowed_vel_)
    {
      max_allowed_vel_ = speed;
    }
  }
  ROS_INFO_THROTTLE(5, "Setting max speed to %f", max_allowed_vel_);

  return true;
}

double SpeedCostFunction::getBearingToObstacle(costmap_2d::ObstructionMsg obs)
{
  // Now calculate the bearing
  return atan2(obs.y, obs.x);
}

double SpeedCostFunction::calculateAllowedSpeed(costmap_2d::ObstructionMsg obs)
{
  // Use circumscribed radius or footprint?
  double distance_to_obstruction = std::max(0.0, std::sqrt(obs.x * obs.x + obs.y * obs.y) - circumscribed_radius_);

  // These should be constants

  double distance_scalar = std::max(0.0,
    std::min(1.0,
      (distance_to_obstruction - min_distance_to_stop_) / (max_distance_to_stop_ - min_distance_to_stop_)));

  return min_linear_velocity_ + (max_linear_velocity_ - min_linear_velocity_) * distance_scalar;
}

void SpeedCostFunction::setCurrentPose(geometry_msgs::PoseStamped pose)
{
  world_frame_id_ = pose.header.frame_id;
  tf::Stamped<tf::Transform> current_pose_tf;
  tf::poseStampedMsgToTF(pose, current_pose_tf);

  current_pose_inv_tf_ = current_pose_tf.inverse();
}


costmap_2d::ObstructionMsg SpeedCostFunction::obstructionToBodyFrame(const costmap_2d::ObstructionMsg& in)
{
  if (in.frame_id == body_frame_id_)
  {
    return in;
  }
  else if (in.frame_id != world_frame_id_)
  {
    ROS_WARN_THROTTLE(1.0, "Received obstruction with unknown frame_id %s", in.frame_id.c_str());
    return in;
  }


  costmap_2d::ObstructionMsg out = in;

  // transform the point
  tf::Vector3 pt_in(in.x, in.y, 0);

  tf::Vector3 pt_out = current_pose_inv_tf_ * pt_in;
  out.x = pt_out[0];
  out.y = pt_out[1];
  return out;
}

void SpeedCostFunction::calculateStopDistances()
{
  max_distance_to_stop_ = 0.5 * max_linear_velocity_ * max_linear_velocity_ / linear_acceleration_;
  min_distance_to_stop_ = 0.5 * min_linear_velocity_ * min_linear_velocity_ / linear_acceleration_;
}

double SpeedCostFunction::scoreTrajectory(Trajectory &traj) {
  if (std::fabs(traj.xv_) > max_allowed_vel_)
  {
    return -1.0;
  }

  return 0.0;
}

} /* namespace base_local_planner */
