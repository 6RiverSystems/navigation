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
#include <algorithm>
#include <costmap_2d/aisle_bias_inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::AisleBiasInflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

AisleBiasInflationLayer::AisleBiasInflationLayer()
  : inflation_radius_(0)
  , weight_(0)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

void AisleBiasInflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
        &AisleBiasInflationLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }

  matchSize();
  need_reinflation_ = true;
}

void AisleBiasInflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor, config.lane_width);

  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
    need_reinflation_ = true;
  }
}

void AisleBiasInflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
}

void AisleBiasInflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (min_x == nullptr || min_y == nullptr || max_x == nullptr || max_y == nullptr)
  {
    ROS_ERROR("Received nullptrs in update bounds callback.  Not updating bounds.");
    return;
  }
  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void AisleBiasInflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;
  ROS_DEBUG("AisleBiasInflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void AisleBiasInflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (!enabled_)
    return;

  ROS_DEBUG_STREAM("VOR update costs ir: " << inflation_radius_ << ", cir: "
    << cell_inflation_radius_ << ", w: " << weight_
    << ", insc: " << inscribed_radius_ << ", res: " << resolution_
    << ", frame: " << layered_costmap_->getGlobalFrameID());

  // Create all of the arrays and queues
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // Arrays to hold the distance calculations
  if (!obstacle_distance_map_)
  {
    obstacle_distance_map_ = std::make_shared<std::vector<double>>();
  }
  obstacle_distance_map_->clear();
  obstacle_distance_map_->resize(size_x * size_y, -1.0);

  // Arrays to hold the angle calculations
  if (!obstacle_angle_map_)
  {
    obstacle_angle_map_ = std::make_shared<std::vector<int>>();
  }
  obstacle_angle_map_->clear();
  obstacle_angle_map_->resize(size_x * size_y, -10);

  double* vor_dist = new double[size_x * size_y];
  std::fill_n(vor_dist, size_x * size_y, -1.0);

  // Maps to act as the queues for the search
  std::map<double, std::vector<CellData> > obs_cells;
  std::map<double, std::vector<CellData> > vor_cells;


  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // First, calculate the distance array.
  // Gather the lethal obstacles
  auto& obs_bin = obs_cells[0.0];
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }
  ROS_DEBUG_STREAM("Gathered " << obs_bin.size() << " obstacles to inflate");


  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (auto& dist_bin: obs_cells)
  {
    for (auto& current_cell: dist_bin.second)
    {
      // process all cells at distance dist_bin.first
      unsigned int index = current_cell.index_;

      // ignore if already visited
      if ((*obstacle_distance_map_)[index] >= -0.1)
      {
        continue;
      }

      // Put the distance for this cell into the array
      (*obstacle_distance_map_)[index] = dist_bin.first;

      unsigned int mx = current_cell.x_;
      unsigned int my = current_cell.y_;
      unsigned int sx = current_cell.src_x_;
      unsigned int sy = current_cell.src_y_;

      // Calculate angle and map to one of 8 headings
      double angle = atan2((double) my - (double) sy, (double) mx - (double)sx);
      int intAngle = 0;
      if (angle < -0.875 * M_PI || angle > 0.875 * M_PI) {
        intAngle = 4;
      } else if (angle < -0.625 * M_PI) {
        intAngle = 5;
      } else if (angle < -0.375 * M_PI) {
        intAngle = 6;       
      } else if (angle < -0.125 * M_PI) {
        intAngle = 7;       
      } else if (angle < 0.125 * M_PI) {
        intAngle = 0;       
      } else if (angle < 0.375 * M_PI) {
        intAngle = 1;       
      } else if (angle < 0.625 * M_PI) {
        intAngle = 2;       
      } else if (angle < -0.875 * M_PI) {
        intAngle = 3;       
      }

      (*obstacle_angle_map_)[index] = intAngle * 45;  // now in degrees
      // If the distance is greater than allowed, mark in voronoi and continue
      // Also, add to the list of voronoi inflation
      if (dist_bin.first > cell_inflation_radius_ + 1)
      {
        vor_cells[0.0].push_back(CellData(index, mx, my, mx, my));
        continue;
      }

      // Check the surrounding cells to see if this is a local maxima.

      if (mx > 0 && mx < size_x -1) {
        if ((*obstacle_distance_map_)[index - 1] <= dist_bin.first
          && (*obstacle_distance_map_)[index - 1] > -0.1
          && (*obstacle_distance_map_)[index + 1] <= dist_bin.first
          && (*obstacle_distance_map_)[index + 1] > -0.1) {
          // Add to the voronoi diagram
          vor_cells[0.0].push_back(CellData(index, mx, my, mx, my));
        }
      }

      if (my > 0 && my < size_y -1) {
        if ((*obstacle_distance_map_)[index - size_x] <= dist_bin.first
          && (*obstacle_distance_map_)[index - size_x] > -0.1
          && (*obstacle_distance_map_)[index + size_x] <= dist_bin.first
          && (*obstacle_distance_map_)[index + size_x] > -0.1) {
          // Add to the voronoi diagram
          vor_cells[0.0].push_back(CellData(index, mx, my, mx, my));
        }
      }

      unsigned int h_inf = std::max(std::abs((int)mx - (int)sx), std::abs((int)my - (int)sy));
      unsigned int dist = dist_bin.first;

      // Calculate the cost for the final cost value.
      // unsigned char cost =  128;
      // if (dist * resolution_ > 0.7) {
      //   unsigned char cost = costLookup((*obstacle_distance_map_)[index], vor_dist[index]);

      // } else {
      unsigned char cost = costLookup(h_inf, 1);
      // unsigned char cost = (unsigned char)(intAngle) * 35;
      

      unsigned char old_cost = master_array[index];
      if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
        master_array[index] = cost;
      else
        master_array[index] = std::max(old_cost, cost);

      // }

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy, obs_cells);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy, obs_cells);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy, obs_cells);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy, obs_cells);
    }
  }

  obs_cells.clear();
  ROS_INFO("Finished obstacle cells.");
  // ROS_INFO_STREAM("Gathered " << vor_cells[0].size() << " voronoi cells to back inflate");
// return;
  // Next, expand the voronoi cells and calculate the actual cost.

  // // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // // can overtake previously inserted but farther away cells
  // for (auto& dist_bin: vor_cells)
  // {
  //   for (auto& current_cell: dist_bin.second)
  //   {
  //     // process all cells at distance dist_bin.first
  //     unsigned int index = current_cell.index_;

  //     // ignore if already visited
  //     if (vor_dist[index] >= -0.1)
  //     {
  //       continue;
  //     }

  //     // If the obstacle distance is < 0, ignore this point
  //     if ((*obstacle_distance_map_)[index] < 0 || master_array[index] == LETHAL_OBSTACLE)
  //     {
  //       continue;
  //     }


  //     // Put the distance for this cell into the array
  //     vor_dist[index] = dist_bin.first;

  //     unsigned int mx = current_cell.x_;
  //     unsigned int my = current_cell.y_;
  //     unsigned int sx = current_cell.src_x_;
  //     unsigned int sy = current_cell.src_y_;

  //     if (std::abs((int)mx - (int)sx) > cell_inflation_radius_ + 2) {
  //       ROS_INFO("od: %f, vd: %f, mx: %d, my %d, sx: %d, sy %d",
  //        (*obstacle_distance_map_)[index], vor_dist[index],
  //        mx, my, sx, sy);
  //     }

  //     unsigned int h_inf = std::max(std::abs(mx - sx), std::abs(my - sy));
  //     unsigned int dist = (*obstacle_distance_map_)[index];

  //     // Calculate the cost for the final cost value.
  //     unsigned char cost =  128;
  //     if (dist * resolution_ > 0.7) {
  //       unsigned char cost = costLookup((*obstacle_distance_map_)[index], vor_dist[index]);

  //     } else {
  //        unsigned char cost = costLookup(h_inf, vor_dist[index]);

  //     }

  //     // unsigned char old_cost = master_array[index];
  //     // if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
  //     //   master_array[index] = cost;
  //     // else
  //     //   master_array[index] = std::max(old_cost, cost);

  //     if (dist_bin.first > cell_inflation_radius_ + 1)
  //     {
  //       continue;
  //     }



  //     // attempt to put the neighbors of the current cell onto the inflation list
  //     if (mx > 0)
  //       enqueue(index - 1, mx - 1, my, sx, sy, vor_cells);
  //     if (my > 0)
  //       enqueue(index - size_x, mx, my - 1, sx, sy, vor_cells);
  //     if (mx < size_x - 1)
  //       enqueue(index + 1, mx + 1, my, sx, sy, vor_cells);
  //     if (my < size_y - 1)
  //       enqueue(index + size_x, mx, my + 1, sx, sy, vor_cells);
  //   }
  // }
  vor_cells.clear();

  // Clean in all up.
  delete[] vor_dist;

}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
inline void AisleBiasInflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y,
                                    std::map<double, std::vector<CellData> >& queue_map)
{
  // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
  double distance = distanceLookup(mx, my, src_x, src_y);


  // push the cell data onto the inflation list and mark
  queue_map[distance].push_back(CellData(index, mx, my, src_x, src_y));
}

void AisleBiasInflationLayer::computeCaches()
{

  ROS_DEBUG_STREAM("VOR Cache compute with ir: " << inflation_radius_ << ", cir: "
    << cell_inflation_radius_ << ", w: " << weight_
    << ", insc: " << inscribed_radius_ << ", res: " << resolution_
    << ", frame: " << layered_costmap_->getGlobalFrameID());
  if (cell_inflation_radius_ == 0)
    return;

  max_effect_distance_ = (lane_width_ + inscribed_radius_) * 2.0;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    deleteKernels();

    cached_costs_.clear();

    cached_distances_ = new double*[cell_inflation_radius_ + 5];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 4; ++i)
    {
      cached_distances_[i] = new double[cell_inflation_radius_ + 5];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 4; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 4; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 4; ++j)
    {
      for (unsigned int ii = 0; ii <= cell_inflation_radius_ + 4; ++ii)
      {
        for (unsigned int jj = 0; jj <= cell_inflation_radius_ + 4; ++jj)
        {
          double d1 = cached_distances_[i][j];
          double d2 = cached_distances_[ii][jj];
          cached_costs_[d1][d2] = computeCost(d1, d2);
        }
      }
    }
  }
}

void AisleBiasInflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 4; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  cached_costs_.clear();
}

void AisleBiasInflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor, double lane_width)
{
  ROS_DEBUG_STREAM("VOR Calling set inflation params with " << inflation_radius << " and " << cost_scaling_factor);
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius 
    || lane_width_ != lane_width)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    lane_width_ = lane_width;
    need_reinflation_ = true;
    ROS_DEBUG_STREAM("About to cache with " << inflation_radius_ << ", "
      << cell_inflation_radius_ << ", " << weight_);
    computeCaches();
  }
  need_reinflation_ = true;
}

}  // namespace costmap_2d
