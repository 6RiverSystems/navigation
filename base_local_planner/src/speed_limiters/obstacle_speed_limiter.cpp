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

#include <base_local_planner/speed_limiters/obstacle_speed_limiter.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/footprint.h>

namespace base_local_planner {

void ObstacleSpeedLimiter::initialize(std::string name) {
  ros::NodeHandle private_nh(name + "/obstacle");
  configServer_ = std::make_shared<dynamic_reconfigure::Server<ObstacleSpeedLimiterConfig>>(private_nh);
  configServer_->setCallback(boost::bind(&ObstacleSpeedLimiter::reconfigure, this, _1, _2));
}

std::string ObstacleSpeedLimiter::getName(){
  return std::string("Obstacle");
}

bool ObstacleSpeedLimiter::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  // Reset the maximum allowed velocity
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  // Collect all of the necessary values
  calculateFootprintBounds(costmap_->getRobotFootprint());

  auto obstructions = costmap_->getLayeredCostmap()->getObstructions();
  if (!obstructions)
  {
    ROS_WARN_THROTTLE(1.0, "No obstructions in speed limiter");
    return false;
  }

  tf::Stamped<tf::Pose> current_pose;
  if (!getCurrentPose(current_pose))
  {
    ROS_WARN_THROTTLE(1.0, "No pose in shadow speed limiter");
    return false;
  }

  tf::Pose current_pose_inv_tf = current_pose.inverse();

  // Find the nearest obstruction to the robot that is within the allowed range
  // Loop over all of the obstructions
  for (const auto& obs : (*obstructions))
  {
    // Skip non-dynamic things or things that have been cleared
    if (obs.type != costmap_2d::ObstructionMsg::DYNAMIC || obs.cleared)
    {
      continue;
    }

    costmap_2d::ObstructionMsg obs_body_frame = obstructionToBodyFrame(obs, current_pose_inv_tf);

    double linear_speed = calculateAllowedLinearSpeed(obs_body_frame);
    if (linear_speed < max_allowed_linear_vel)
    {
      max_allowed_linear_vel = linear_speed;
    }

    double angular_speed = calculateAllowedAngularSpeed(obs_body_frame);
    if (angular_speed < max_allowed_angular_vel)
    {
      max_allowed_angular_vel = angular_speed;
    }
  }
  ROS_DEBUG_THROTTLE(0.2, "Setting max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);

  return true;
}

double ObstacleSpeedLimiter::getBearingToObstacle(const costmap_2d::ObstructionMsg& obs)
{
  return atan2(obs.y, obs.x);
}

double ObstacleSpeedLimiter::calculateAllowedLinearSpeed(const costmap_2d::ObstructionMsg& obs)
{
  // Check if the bearing to the obstacle is acceptable
  if (std::fabs(getBearingToObstacle(obs)) > params_.half_angle)
  {
    return max_linear_velocity_;
  }

  double abs_y_dist = 0;
  if (obs.y < footprint_min_y_)
  {
    abs_y_dist = footprint_min_y_ - obs.y;
  }
  else if (obs.y > footprint_max_y_)
  {
    abs_y_dist = obs.y - footprint_max_y_;
  }

  double abs_x_dist = 0;
  if (obs.x < footprint_min_x_)
  {
    abs_x_dist = footprint_min_x_ - obs.x;
  }
  else if (obs.x > footprint_max_x_)
  {
    abs_x_dist = obs.x - footprint_max_x_;
  }

  double x_dist_with_buffer = std::max(0.0, abs_x_dist - params_.x_buffer);
  double y_dist_with_buffer = std::max(0.0, abs_y_dist - params_.y_buffer);

  double distance_to_obstruction = std::sqrt(x_dist_with_buffer * x_dist_with_buffer + y_dist_with_buffer * y_dist_with_buffer);
  ROS_DEBUG("Obs: %f, %f.  abs x: %f, abs y: %f, Dist: %f", obs.x, obs.y, abs_x_dist, abs_y_dist, distance_to_obstruction);

  double speed = 0.0;
  if (distance_to_obstruction < .2)
  {
    speed = 0.45;
  }
  else 
  {
    speed = math.pow(distance_to_obstruction,1.0/1.172);
  }
  if(speed > 1.3){
    speed = 1.3;
  }
}


double ObstacleSpeedLimiter::calculateAllowedAngularSpeed(const costmap_2d::ObstructionMsg& obs)
{
  double distance_to_obstruction = std::sqrt(obs.x * obs.x + obs.y * obs.y) - circumscribed_radius_;

  return twoLevelInterpolation(distance_to_obstruction, 
    params_.min_angular_velocity_effect_distance, params_.max_angular_velocity_effect_distance,
    std::min(params_.min_angular_velocity, max_angular_velocity_), 
    max_angular_velocity_);
}


costmap_2d::ObstructionMsg ObstacleSpeedLimiter::obstructionToBodyFrame(const costmap_2d::ObstructionMsg& in,
  const tf::Pose& current_pose_inv_tf)
{
  if (in.frame_id == costmap_->getBaseFrameID())
  {
    return in;
  }
  else if (in.frame_id != costmap_->getGlobalFrameID())
  {
    ROS_ERROR_THROTTLE(1.0, "Received obstruction with unknown frame_id %s", in.frame_id.c_str());
    return in;
  }

  costmap_2d::ObstructionMsg out = in;

  // transform the point
  tf::Vector3 pt_in(in.x, in.y, 0);

  tf::Vector3 pt_out = current_pose_inv_tf * pt_in;
  out.x = pt_out[0];
  out.y = pt_out[1];

  ROS_DEBUG("Converted point from [%f, %f] to [%f, %f]", in.x, in.y, out.x, out.y);

  return out;
}

void ObstacleSpeedLimiter::calculateFootprintBounds(const std::vector<geometry_msgs::Point>& footprint)
{
  if (footprint.empty())
  {
    ROS_ERROR_THROTTLE(1.0, "Trying to calculate footprint bounds in speed cost function, but footprint is empty");
    // Set 0 values
    footprint_max_x_ = 0;
    footprint_max_y_ = 0;
    footprint_min_y_ = 0;
    footprint_min_x_ = 0;

    circumscribed_radius_ = 0;
  }
  else
  {
    // Reset the bounds
    footprint_max_x_ = -std::numeric_limits<double>::max();
    footprint_max_y_ = -std::numeric_limits<double>::max();
    footprint_min_y_ = std::numeric_limits<double>::max();
    footprint_min_x_ = std::numeric_limits<double>::max();

    circumscribed_radius_ = 0;

    for (auto pt : footprint)
    {
      if (pt.x < footprint_min_x_) {footprint_min_x_ = pt.x;}
      if (pt.x > footprint_max_x_) {footprint_max_x_ = pt.x;}
      if (pt.y < footprint_min_y_) {footprint_min_y_ = pt.y;}
      if (pt.y > footprint_max_y_) {footprint_max_y_ = pt.y;}
    }
  }

  double inscribed_radius = 0.0;

  costmap_2d::calculateMinAndMaxDistances(footprint, inscribed_radius, circumscribed_radius_);
}

} /* namespace base_local_planner */
