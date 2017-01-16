/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2017, 6 River Systems, Inc.
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
 *         Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_OBSTRUCTION_LAYER_H_
#define COSTMAP_2D_OBSTRUCTION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstructionPluginConfig.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/ObstructionMsg.h>
#include <costmap_2d/ObstructionListMsg.h>


namespace costmap_2d
{


/////////////////////////////////////////////////////////////
// HELPER CLASSES
/////////////////////////////////////////////////////////////
struct Obstruction
{
  // Obstruction(){};

  Obstruction(float x, float y, std::string frame) : x_(x), y_(y), frame_(frame),
    first_sighting_time_(ros::Time::now()), last_sighting_time_(ros::Time::now()),
    last_level_time_(ros::Time::now()) {}

  void touch()
  {
    last_sighting_time_ = ros::Time::now();
    last_level_time_ = last_sighting_time_;
    cleared_ = false;
    level_ = 0;
  }

  ObstructionMsg asMsg()
  {
    ObstructionMsg msg;
    msg.x = x_;
    msg.y = y_;
    msg.last_sighting_time = last_sighting_time_;
    msg.first_sighting_time = first_sighting_time_;
    msg.cleared = cleared_;
    msg.effective_radius = radius_;
    msg.frame_id = frame_;

    return msg;
  }

  ros::Time first_sighting_time_ = ros::Time(0); // time obstruction was first seen
  ros::Time last_sighting_time_ = ros::Time(0);  // last time obstruction was seen
  ros::Time last_level_time_ = ros::Time(0);  // last time obstruction changed levels
  float x_ = 0.0; // x location of obstruction
  float y_ = 0.0; // y location of obstruction
  unsigned int level_ = 0; // obstruction level
  bool cleared_ = false; // flag to indicate if the obstruction is cleared and should be removed
  float radius_ = -1;
  std::string frame_ = "";
};


class Kernel
{
public:
  Kernel() : values_(nullptr) {};

  ~Kernel()
  {
    if (values_) { delete[] values_;}
  };

  float getRadius()
  {
    return radius_;
  }

  /// TODO: move to cpp file
  void generateRadialInflationKernel(unsigned char max_value, unsigned char inscribed_value,
    double inscribed_radius, double inflation_radius, double cost_scaling_factor, double resolution)
  {
    resolution_ = resolution;
    radius_ = inflation_radius;

    unsigned int cell_inflation_radius = (unsigned int) std::max(0.0, ceil(inflation_radius / resolution));
    unsigned int size =  cell_inflation_radius * 2 + 1;
    size_x_ = size;
    size_y_ = size;
    unsigned int center_x = size_x_ / 2 + 1;
    unsigned int center_y = size_y_ / 2 + 1;

    if (values_) { delete[] values_;}
    values_ = new unsigned char[size_x_ * size_y_];

    for (unsigned int yy = 0; yy < size_y_; ++yy)
    {
      for (unsigned int xx = 0; xx < size_x_; ++xx)
      {
        double distance = std::hypot(center_x - xx, center_y - yy);

        unsigned char cost = 0;
        if (distance == 0)
        {
          cost = max_value;
        }
        else if (distance * resolution <= inscribed_radius)
        {
          cost = inscribed_value;
        }
        else if (inscribed_value > 0 && distance * resolution <= inflation_radius)
        {
          double factor = exp(-1.0 * cost_scaling_factor * (distance - inscribed_radius));
          cost = (unsigned char)((inscribed_value - 1) * factor);
        }

        values_[size_x_ * yy + xx] = cost;
      }
    }
  }

  void applyKernelAtLocation(float x, float y, Costmap2D& master_grid)
  {
    // Get the grid location of the point.
    int mx, my;
    master_grid.worldToMapNoBounds(x, y, mx, my);

    unsigned int grid_x_size = master_grid.getSizeInCellsX();
    unsigned int grid_y_size = master_grid.getSizeInCellsY();

    unsigned int center_x = size_x_ / 2 + 1;
    unsigned int center_y = size_y_ / 2 + 1;
    // Now try to apply it.
    for (unsigned int yy = 0; yy < size_y_; ++yy)
    {
      unsigned int grid_y = yy - center_y + my;
      if (grid_y < grid_y_size && grid_y >= 0)
      {
        for (unsigned int xx = 0; xx < size_x_; ++xx)
        {
          unsigned int grid_x = xx - center_x + mx;
          if (grid_x < grid_x_size && grid_x >= 0)
          {
            // Point is valid, max it in.
            unsigned char grid_cost = master_grid.getCost(grid_x, grid_y);
            unsigned char kernel_cost = values_[size_x_ * yy + xx];
            if (grid_cost < kernel_cost || grid_cost == NO_INFORMATION)
            {
              master_grid.setCost(grid_x, grid_y, kernel_cost);
            }
          }
        }
      }
    }
  }

private:
  unsigned int size_x_ = 1; // size
  unsigned int size_y_ = 1; // size
  unsigned char* values_; // square array that should get centered on obstruction
  float resolution_ = 1; // resolution of the costmap
  float radius_ = 1; // radius of affect of the costmap
};

class ClearObstructionCell
{
public:
  ClearObstructionCell(std::shared_ptr<Obstruction>* costmap) :
      costmap_(costmap)
  {
  }
  inline void operator()(unsigned int offset)
  {
    std::shared_ptr<Obstruction> obs = costmap_[offset];
    if (obs)
    {
      obs->cleared_ = true;
      costmap_[offset].reset();
    }
  }
private:
  std::shared_ptr<Obstruction>* costmap_;
};


class ObstructionLayer : public CostmapLayer
{
public:
  ObstructionLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    obstruction_map_ = NULL;
  }

  virtual ~ObstructionLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                         const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

   /**
    * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
    * @param message The message returned from a message notifier
    * @param buffer A pointer to the observation buffer to update
    */
  void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                                 const boost::shared_ptr<ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                          const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud2 messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  // for testing purposes
  void addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);


  virtual void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);

  virtual void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);


protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param clearing_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;

  /**
   * @brief  Clear freespace based on one observation
   * @param clearing_observation The observation used to raytrace
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  virtual void checkObservations(const costmap_2d::Observation& observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  virtual void updateObstructions(double* min_x, double* min_y, double* max_x, double* max_y);


  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                       double* max_x, double* max_y);


  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  virtual void deleteMaps();

  /**
   * @brief  Resets the costmap and static_map to be unknown space
   */
  virtual void resetMaps();

  /**
   * @brief  Initializes the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  double max_obstacle_height_;  ///< @brief Max Obstacle Height

  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  // Used only for testing purposes
  std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ObstructionPluginConfig> *dsrv_;

  // Bits that are new for obstuctions
  std::shared_ptr<Obstruction>* obstruction_map_;
  std::list<std::shared_ptr<Obstruction>> obstruction_list_; /// @todo update to priority queue sorted on last time

  // Things from the inflation layer
  std::vector<std::shared_ptr<Kernel>> kernels_; // vector of kernels for different obstruction levels

  ros::Duration obstruction_half_life_; // The time to wait before decrementing the obstruction level by half.
  unsigned int num_obstruction_levels_;  // The number of levels the obstruction should go through before disappearing
  float inflation_radius_;
  float cost_scaling_factor_;

  ros::Publisher obstruction_publisher_;  // Publisher of obstruction data

  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

  // Generate all of the kernels
  void generateKernels();

private:
  void reconfigureCB(costmap_2d::ObstructionPluginConfig &config, uint32_t level);
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_OBSTACLE_LAYER_H_
