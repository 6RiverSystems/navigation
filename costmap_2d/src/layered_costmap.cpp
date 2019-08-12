/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <costmap_2d/costmap_layer.h>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown) :
    costmap_(), global_frame_(global_frame), rolling_window_(rolling_window), initialized_(false), size_locked_(false), timingDataRecorder_("layered"+global_frame)
{
  if (track_unknown)
    costmap_.setDefaultValue(255);
  else
    costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

void LayeredCostmap::addPlugin(boost::shared_ptr<CostmapLayer> plugin)
{
  if (plugin->getLayerType() == LayerType::STATIC_IMPASSIBLE
    && !static_layer_) {
    static_layer_ = plugin;
  } else if (plugin->getLayerType() == LayerType::OBSTRUCTION) {
    obstruction_layers_.push_back(plugin);
  } else if (plugin->getLayerType() == LayerType::SHADOW) {
    shadow_layer_ = plugin;
  }
  plugins_.push_back(plugin);
}

std::shared_ptr<std::vector<ObstructionMsg>> LayeredCostmap::getObstructions()
{
  auto output = std::make_shared<std::vector<ObstructionMsg>>();

  for (auto layer : obstruction_layers_)
  {
    auto obs = layer->getObstructions();
    if (obs)
    {
      output->insert(output->end(), obs->begin(), obs->end());
    }
  }

  return output;
}

std::shared_ptr<std::vector<geometry_msgs::Point>> LayeredCostmap::getShadowedObjects()
{
  if (shadow_layer_)
  {
    return shadow_layer_->getShadowedObjects();
  }
  else
  {
    return std::shared_ptr<std::vector<geometry_msgs::Point>>();
  }
}

std::shared_ptr<std::vector<double>> LayeredCostmap::getDistancesFromStaticMap()
{
  if (static_layer_) {
    return static_layer_->getDistancesFromStaticMap();
  } else {
    return std::shared_ptr<std::vector<double>>();
  }
}

double LayeredCostmap::getDistanceFromStaticMap(double px, double py)
{
  if (static_layer_)
  {
    return static_layer_->getDistanceFromStaticMap(px, py);
  }
  else
  {
    return -1.0;
  }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<CostmapLayer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.

  srs::TimingDataRecorder* rec = nullptr;
  if (rolling_window_) {rec = timingDataRecorder_.getRecorder("-getLock", 1);  rec->startSample();}

  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  if (rec) {rec->stopSample();}

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;

  rec = nullptr;
  if (rolling_window_) {rec = timingDataRecorder_.getRecorder("-updateBounds", 1);  rec->startSample();}

  for (vector<boost::shared_ptr<CostmapLayer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }
  if (rec) {rec->stopSample();}

  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);
  if (xn < x0 || yn < y0){
    return;
  }

  rec = nullptr;
  if (rolling_window_) {rec = timingDataRecorder_.getRecorder("-resetMaps", 1);  rec->startSample();}

  costmap_.resetMap(x0, y0, xn, yn);
  if (rec) {rec->stopSample();}

  rec = nullptr;
  if (rolling_window_) {rec = timingDataRecorder_.getRecorder("-updateCosts", 1);  rec->startSample();}

  for (vector<boost::shared_ptr<CostmapLayer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    srs::TimingDataRecorder* rec2 = nullptr;
    if (rolling_window_) {rec2 = timingDataRecorder_.getRecorder("-updateCost"+(*plugin)->getName(), 1);  rec2->startSample();}

    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
    if (rec2) {rec2->stopSample();}
  }
  if (rec) {rec->stopSample();}

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<CostmapLayer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<CostmapLayer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
