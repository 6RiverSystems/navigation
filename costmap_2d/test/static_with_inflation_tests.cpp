/*
 * Copyright (c) 2017, 6 River Systems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Daniel Grieneisen, Sindhura Chayapathy
 * Test harness for StaticLayerWithInflation for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>
#include <srslib_timing/StopWatch.hpp>
#include <set>
#include <random>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */

const unsigned int BIGMAP_OBSTACLES = 47790;
const unsigned int NUM_UPDATES = 3;
const unsigned int NUM_OBSTACLES = 10;
const float OBSTRUCTION_SPREAD = 20.0;

/**
 * Time map updates.
 */
TEST(costmap, testTimeMapUpdatesNominal) {
  // Create a normal layered costmap.
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  std::vector<geometry_msgs::Point> polygon = setRadii(layers, 1, 1.75, 3);
  layers.setFootprint(polygon);

  // Robot location
  float rx = 32.2;
  float ry = 47.0;
  float rth = 0.0;

  std::cerr << "Starting old typefirst update." << std::endl;
  srs::StopWatch sw = srs::StopWatch();
  layers.updateMap(rx, ry, rth);
  std::cerr << "Time for first update " << sw.elapsedMicroseconds() << std::endl;

  sw.reset();
  layers.updateMap(rx, ry, rth);
  std::cerr << "Time for second update " << sw.elapsedMicroseconds() << std::endl;
  layers.updateMap(rx, ry, rth);

  // obstacle count == static layer obstacle count
  Costmap2D* costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), BIGMAP_OBSTACLES);

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-OBSTRUCTION_SPREAD, OBSTRUCTION_SPREAD);
  sw.reset();
  for (unsigned int k = 0; k < NUM_UPDATES; ++k) {
   
    for (unsigned int jj = 0; jj < NUM_OBSTACLES; ++jj)
    {
      float obs_x;
      float obs_y;
      
      unsigned char cost = costmap_2d::LETHAL_OBSTACLE;
      while (cost == costmap_2d::LETHAL_OBSTACLE) {
        obs_x = rx + distribution(generator);
        obs_y = ry + distribution(generator);
        unsigned int map_obs_x, map_obs_y;
        layers.getCostmap()->worldToMap(obs_x, obs_y, map_obs_x, map_obs_y);
        cost = layers.getCostmap()->getCost(map_obs_x, map_obs_y); 
      }
      addObservation(olayer, obs_x, obs_y, 0.1, rx, ry);
    }
    layers.updateMap(rx, ry, rth);
    olayer->clearStaticObservations(true, true);
  }
  std::cerr << "Time for next updates: " << (double)sw.elapsedMicroseconds() / NUM_UPDATES << " ms per update" << std::endl;

  // Print result and verify the correct number of obstacles
  costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), NUM_OBSTACLES * NUM_UPDATES + BIGMAP_OBSTACLES);
}

/**
 * Time map updates.
 */
TEST(costmap, testTimeMapUpdatesNew) {
  // Create a normal layered costmap.
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayerWithInflation(layers, tf);

  ObstructionLayer* olayer = addObstructionLayer(layers, tf);

  std::vector<geometry_msgs::Point> polygon = setRadii(layers, 1, 1.75, 3);
  layers.setFootprint(polygon);

  // Robot location
  float rx = 32.2;
  float ry = 47.0;
  float rth = 0.0;

  std::cerr << "Starting new type first update." << std::endl;
  srs::StopWatch sw = srs::StopWatch();
  layers.updateMap(rx, ry, rth);
  std::cerr << "Time for first update " << sw.elapsedMicroseconds() << std::endl;

  sw.reset();
  layers.updateMap(rx, ry, rth);
  std::cerr << "Time for second update " << sw.elapsedMicroseconds() << std::endl;
  layers.updateMap(rx, ry, rth);
  // obstacle count == static layer obstacle count
  Costmap2D* costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), BIGMAP_OBSTACLES);

  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-OBSTRUCTION_SPREAD, OBSTRUCTION_SPREAD);
  sw.reset();
  for (unsigned int k = 0; k < NUM_UPDATES; ++k) {
    for (unsigned int jj = 0; jj < NUM_OBSTACLES; ++jj)
    {
      float obs_x;
      float obs_y;
      
      unsigned char cost = costmap_2d::LETHAL_OBSTACLE;
      while (cost == costmap_2d::LETHAL_OBSTACLE) {
        obs_x = rx + distribution(generator);
        obs_y = ry + distribution(generator);
        unsigned int map_obs_x, map_obs_y;
        layers.getCostmap()->worldToMap(obs_x, obs_y, map_obs_x, map_obs_y);
        cost = layers.getCostmap()->getCost(map_obs_x, map_obs_y);
      }
      addObservation(olayer, obs_x, obs_y, 0.1, rx, ry);
    }
    layers.updateMap(rx, ry, rth);
    olayer->clearStaticObservations(true, true);
  }
  std::cerr << "Time for next updates " << (double)sw.elapsedMicroseconds() / NUM_UPDATES << " ms per update" << std::endl;

  // Print result and verify the correct number of obstacles
  costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), BIGMAP_OBSTACLES + NUM_UPDATES * NUM_OBSTACLES);
}

/**
 * Tests the onInitialize method
 */
TEST(costmap, StaticLayerWithInflationMapOnInitialize){

  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayerWithInflation(layers, tf);  // This adds the StaticLayerWithInflation map

  std::vector<boost::shared_ptr<CostmapLayer>>* plugin = layers.getPlugins();
  ASSERT_EQ((*(plugin->begin()))->layerInitialized(), 1);
  ASSERT_EQ(layers.areAllLayersInitialized(), 1);
}

/**
 * Tests the deactivate method
 */
TEST(costmap, StaticLayerWithInflationMapDeactivated){

  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayerWithInflation(layers, tf);  // This adds the StaticInflation map
  std::vector<boost::shared_ptr<CostmapLayer>>* plugin = layers.getPlugins();
  // call deactivate function
  (*(plugin->begin()))->deactivate();
  ASSERT_EQ((*(plugin->begin()))->layerInitialized(), 0);
  ASSERT_EQ(layers.areAllLayersInitialized(), 0);
}

/**
 * Tests the deactivate and activate method
 */
TEST(costmap, StaticLayerWithInflationMapDeactivateAndActivate){

  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayerWithInflation(layers, tf);  // This adds the StaticInflation map

  std::vector<boost::shared_ptr<CostmapLayer>>* plugin = layers.getPlugins();
  (*(plugin->begin()))->deactivate();
  (*(plugin->begin()))->activate();

  ASSERT_EQ((*(plugin->begin()))->layerInitialized(), 1);
  ASSERT_EQ(layers.areAllLayersInitialized(), 1);
}

/**
 * Tests the reset method
 */
TEST(costmap, StaticLayerWithInflationMapReset){

  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayerWithInflation(layers, tf);  // This adds the StaticLayerWithInflation map

  std::vector<boost::shared_ptr<CostmapLayer>>* plugin = layers.getPlugins();

  (*(plugin->begin()))->reset();

  ASSERT_EQ((*(plugin->begin()))->layerInitialized(), 1);
  ASSERT_EQ(layers.areAllLayersInitialized(), 1);
}

/**
 * Tests the reinitialize method
 */
TEST(costmap, StaticLayerWithInflationMapReinitialize){

  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);  // Not rolling window, not tracking unknown
  addStaticLayerWithInflation(layers, tf);  // This adds the StaticLayerWithInflation map

  std::vector<boost::shared_ptr<CostmapLayer>>* plugin = layers.getPlugins();

  (*(plugin->begin()))->reinitialize();

  ASSERT_EQ((*(plugin->begin()))->layerInitialized(), 1);
  ASSERT_EQ(layers.areAllLayersInitialized(), 1);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "inflation_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
